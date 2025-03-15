use core::ops::{Deref, DerefMut};
use esp_hal::dma::DmaRxStreamBuf;
use esp_hal::i2s::master::camera::{Camera, CameraTransfer};
use esp_hal::peripherals::I2S0;

pub const BUF_SIZE: usize = 20 * 1000;

pub struct MyCamera<'a> {
    state: DriverState<'a>,
}

impl<'d> MyCamera<'d> {
    pub fn new(camera: Camera<'d, I2S0>, buf: DmaRxStreamBuf) -> Self {
        Self {
            state: DriverState::Idle(camera, buf),
        }
    }

    pub fn receive<'a>(&'a mut self) -> MyCamTransfer<'a, 'd> {
        let state = core::mem::take(&mut self.state);
        let DriverState::Idle(camera, buf) = state else {
            unreachable!()
        };

        let transfer = camera.receive(buf, BUF_SIZE).unwrap();
        self.state = DriverState::Running(transfer);

        MyCamTransfer { driver: self }
    }
}

#[derive(Default)]
enum DriverState<'d> {
    Idle(Camera<'d, I2S0>, DmaRxStreamBuf),
    Running(CameraTransfer<'d, I2S0, DmaRxStreamBuf>),
    #[default]
    Borrowed,
}

pub struct MyCamTransfer<'a, 'd> {
    driver: &'a mut MyCamera<'d>,
}

impl<'d> Deref for MyCamTransfer<'_, 'd> {
    type Target = CameraTransfer<'d, I2S0, DmaRxStreamBuf>;

    fn deref(&self) -> &Self::Target {
        match &self.driver.state {
            DriverState::Running(transfer) => transfer,
            _ => unreachable!(),
        }
    }
}

impl DerefMut for MyCamTransfer<'_, '_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match &mut self.driver.state {
            DriverState::Running(transfer) => transfer,
            _ => unreachable!(),
        }
    }
}

impl Drop for MyCamTransfer<'_, '_> {
    fn drop(&mut self) {
        let state = core::mem::take(&mut self.driver.state);

        let DriverState::Running(transfer) = state else {
            unreachable!()
        };

        let (camera, buf) = transfer.stop();
        self.driver.state = DriverState::Idle(camera, buf);
    }
}
