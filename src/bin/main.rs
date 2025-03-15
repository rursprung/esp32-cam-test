#![feature(iter_from_coroutine)]
#![feature(coroutines)]
#![feature(allocator_api)]
#![no_std]
#![no_main]

use alloc::string::ToString;
use alloc::vec::Vec;
use core::cell::RefCell;
use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_net::{Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use esp_hal::peripherals::I2S0;
use esp_hal::psram::psram_raw_parts;
use esp_hal::rng::Rng;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma_rx_stream_buffer,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    i2s::master::camera::{self, Camera},
    ledc::{
        channel::{self, config::PinConfig, ChannelIFace},
        timer,
        timer::{config::Duty, LSClockSource, Number, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println as _;
use esp_wifi::config::PowerSaveMode;
use esp_wifi::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState,
};
use esp_wifi::EspWifiController;
use picoserve::response::chunked::{ChunkWriter, ChunkedResponse, Chunks, ChunksWritten};
use picoserve::response::{Content, Response};
use picoserve::routing::get;
use static_cell::{ConstStaticCell, StaticCell};

use esp32_cam_test::camera::*;
use esp32_cam_test::ov2640::Ov2640;

use esp_backtrace as _;

extern crate alloc;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

static PSRAM_HEAP: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();

const BUF_SIZE: usize = 20 * 1000;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let (start, size) = psram_raw_parts(&peripherals.PSRAM);
    unsafe { PSRAM_HEAP.init(start as _, size) };

    esp_alloc::heap_allocator!(size: 96 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let mut delay = Delay::new();

    // camera init
    let cam_siod = peripherals.GPIO26;
    let cam_sioc = peripherals.GPIO27;
    let cam_xclk = peripherals.GPIO0;
    // power down pin - must be pulled low for the camera to run!
    let _cam_pwdn = Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default());
    let camera = Camera::new(
        peripherals.I2S0,
        camera::Config::default(),
        peripherals.DMA_I2S0,
    )
    .with_ws(peripherals.GPIO22)
    .with_vsync(peripherals.GPIO25)
    .with_hsync(peripherals.GPIO23)
    .with_data_pins(
        peripherals.GPIO5,
        peripherals.GPIO18,
        peripherals.GPIO19,
        peripherals.GPIO21,
        peripherals.GPIO36,
        peripherals.GPIO39,
        peripherals.GPIO34,
        peripherals.GPIO35,
    );
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut timer = ledc.timer::<LowSpeed>(Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: Duty::Duty1Bit,
            clock_source: LSClockSource::APBClk,
            frequency: Rate::from_mhz(20),
        })
        .unwrap();
    let mut channel = ledc.channel(channel::Number::Channel0, cam_xclk);
    channel
        .configure(channel::config::Config {
            timer: &timer,
            duty_pct: 50,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();
    delay.delay_millis(500u32);
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(cam_siod)
        .with_scl(cam_sioc);
    let _ov2640 = Ov2640::new(i2c, &mut delay).unwrap();

    // WIFI init
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);
    let seed = rng.random() as u64;
    let init = esp_wifi::init(timer1.timer0, rng, peripherals.RADIO_CLK).unwrap();
    let init = &*{
        static WIFI_INIT: StaticCell<EspWifiController<'static>> = StaticCell::new();
        WIFI_INIT.init(init)
    };
    let (controller, wifi_interfaces) = esp_wifi::wifi::new(init, peripherals.WIFI).unwrap();
    let stack_resources = {
        static STACK_RESOURCES: ConstStaticCell<StackResources<3>> =
            ConstStaticCell::new(StackResources::new());
        STACK_RESOURCES.take()
    };
    let (stack, runner) = embassy_net::new(
        wifi_interfaces.sta,
        embassy_net::Config::dhcpv4(Default::default()),
        stack_resources,
        seed,
    );

    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();

    stack.wait_link_up().await;

    info!("Waiting to get IP address...");

    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", &config.address);
            break;
        }
    }

    run_server(stack, camera).await;
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    info!(
        "Device capabilities: {}",
        defmt::Debug2Format(&controller.capabilities())
    );

    controller.set_power_saving(PowerSaveMode::None).unwrap();

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });

    loop {
        if let WifiState::StaConnected = esp_wifi::wifi::wifi_state() {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }

        if !matches!(controller.is_started(), Ok(true)) {
            controller.set_configuration(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut stack: embassy_net::Runner<'static, WifiDevice<'static>>) {
    stack.run().await
}

async fn run_server(stack: Stack<'static>, camera: Camera<'static, I2S0>) {
    let dma_rx_buf = dma_rx_stream_buffer!(BUF_SIZE, 1000);
    let camera = RefCell::new(MyCamera::new(camera, dma_rx_buf));

    let config = picoserve::Config::new(picoserve::Timeouts {
        start_read_request: Some(Duration::from_secs(5)),
        read_request: Some(Duration::from_secs(1)),
        write: Some(Duration::from_secs(1)),
    });

    let router = picoserve::Router::new()
        .route(
            "/image",
            get(|| async {
                info!("serving /image");
                let mut stream = ImageStream {
                    camera: &camera,
                    image_count: 0,
                };
                Response::ok(stream.next_jpeg())
            }),
        )
        .route(
            "/stream",
            get(|| async {
                info!("serving /stream");
                ChunkedResponse::new(ImageStream {
                    camera: &camera,
                    image_count: 0,
                })
            }),
        );

    let mut tx_buf = Vec::new_in(&PSRAM_HEAP);
    tx_buf.resize(10 * 1024, 0);

    info!(
        "now serving HTTP requests on http://{}/stream, http://{}/image and http://{}/hello",
        stack.config_v4().unwrap().address.address(),
        stack.config_v4().unwrap().address.address(),
        stack.config_v4().unwrap().address.address(),
    );

    picoserve::listen_and_serve(
        0,
        &router,
        &config,
        stack,
        80,
        &mut [0; 1024],
        // &mut [0; 1024 * 40],
        &mut tx_buf,
        &mut [0; 2048],
    )
    .await;
}

struct ImageStream<'ch> {
    camera: &'ch RefCell<MyCamera<'static>>,
    image_count: usize,
}

impl ImageStream<'_> {
    fn next_jpeg(&mut self) -> Image {
        let mut camera = self.camera.borrow_mut();
        let mut transfer = camera.receive();

        let mut byte_stream = core::iter::from_coroutine(
            #[coroutine]
            || {
                loop {
                    let (data, _ends_with_eof) = transfer.peek_until_eof();
                    if data.is_empty() {
                        if transfer.is_done() {
                            error!("Too slow in reading image data!");
                        }
                    } else {
                        let bytes_peeked = data.len();
                        // Convert [FF, 00, FF, 00, D8, 00, D8, 00, FF, 00, FF, 00, E0, 00, E0] to [FF,
                        // D8, FF, E0]
                        for i in 0..(bytes_peeked / 4) {
                            yield data[i * 4]
                        }
                        transfer.consume(bytes_peeked);
                    }
                    if _ends_with_eof {
                        debug!("found EOF, ignoring");
                    }
                }
            },
        );

        debug!("looking for JPEG header...");

        let mut jpegs_skipped = 0;
        let mut bytes_skipped = 0;
        loop {
            let byte: u8 = byte_stream.next().unwrap();
            if byte != 0xFF {
                bytes_skipped += 1;
                continue;
            }
            let byte: u8 = byte_stream.next().unwrap();
            if byte != 0xD8 {
                bytes_skipped += 2;
                continue;
            }
            let byte: u8 = byte_stream.next().unwrap();
            if byte != 0xFF {
                bytes_skipped += 3;
                continue;
            }
            let byte: u8 = byte_stream.next().unwrap();
            if byte != 0xE0 {
                bytes_skipped += 4;
                continue;
            }

            // We've found the starting JPEG marker of FF, D8, FF, E0.

            // We want to skip the first 10 of these as they're likely to be garbage.
            if jpegs_skipped < 10 {
                bytes_skipped += 4;
                jpegs_skipped += 1;
                continue;
            }

            // found JPEG
            break;
        }
        debug!("skipped {} bytes", bytes_skipped);

        let mut frame = Vec::<u8>::with_capacity(10000);

        frame.push(0xFF);
        frame.push(0xD8);
        frame.push(0xFF);
        frame.push(0xE0);
        let mut idx = 4;

        loop {
            if idx % 1000 == 0 {
                debug!("read {} bytes so far, still looking for the end", idx);
            }
            let byte: u8 = byte_stream.next().unwrap();
            frame.push(byte);
            idx += 1;
            if byte != 0xFF {
                continue;
            }
            let byte: u8 = byte_stream.next().unwrap();
            frame.push(byte);
            idx += 1;
            if byte != 0xD9 {
                continue;
            }

            // We've found the ending JPEG marker of FF, D9.
            break;
        }
        debug!("found the end after {} bytes", idx);
        frame.truncate(idx);

        if self.image_count == usize::MAX {
            self.image_count = 0;
        } else {
            self.image_count += 1;
        }

        Image { image: frame }
    }
}

impl Chunks for ImageStream<'_> {
    fn content_type(&self) -> &'static str {
        "multipart/x-mixed-replace;boundary=123456789000000000000987654321"
    }

    async fn write_chunks<W: Write>(
        mut self,
        mut chunk_writer: ChunkWriter<W>,
    ) -> Result<ChunksWritten, W::Error> {
        loop {
            chunk_writer
                .write_chunk(b"\r\n--123456789000000000000987654321\r\n")
                .await?;
            chunk_writer
                .write_chunk(b"Content-Type: image/jpeg\r\n")
                .await?;
            chunk_writer
                .write_chunk(
                    ("X-Frame-Id: ".to_string()
                        + &*self.image_count.to_string()
                        + &*"\r\n\r\n".to_string())
                        .as_bytes(),
                )
                .await?;

            chunk_writer.write_chunk(&self.next_jpeg().image).await?;
            chunk_writer.flush().await?;
        }
    }
}

struct Image {
    image: Vec<u8>,
}

impl Content for Image {
    fn content_type(&self) -> &'static str {
        "image/jpeg"
    }

    fn content_length(&self) -> usize {
        self.image.len()
    }

    async fn write_content<W: Write>(self, mut writer: W) -> Result<(), W::Error> {
        writer.write_all(&self.image).await
    }
}
