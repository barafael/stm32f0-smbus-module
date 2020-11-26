//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

use stm32f0xx_hal::gpio::gpiob::PB8;
use stm32f0xx_hal::gpio::gpiob::PB9;
use stm32f0xx_hal::gpio::Alternate;
use stm32f0xx_hal::gpio::AF1;

use stm32f0xx_hal::{
    gpio::gpioa::PA5,
    gpio::gpioc::PC13,
    gpio::{Floating, Input, Output, PushPull},
    pac,
    prelude::*,
};

use cortex_m::interrupt::free as disable_interrupts;

#[derive(Debug)]
enum SMBusDirection {
    SlaveToMaster,
    MasterToSlave,
}

impl Default for SMBusDirection {
    fn default() -> Self {
        SMBusDirection::MasterToSlave
    }
}

const transmission_size: usize = 32;

#[derive(Default, Debug)]
pub struct SMBusState {
    dir: SMBusDirection,
    dataReady: bool,
    bytesToTransmit: usize,
    receive_buffer: [u8; transmission_size],
    transmit_buffer: [u8; transmission_size],
}

#[app(device = stm32f0xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        exti: pac::EXTI,
        user_button: PC13<Input<Floating>>,
        led: PA5<Output<PushPull>>,
        i2c: pac::I2C1,
        transmissionState: SMBusState,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        // RTT handler
        rtt_init_print!();

        // Alias peripherals
        let mut dp: pac::Peripherals = ctx.device;

        rprintln!("Initializing clocks");

        dp.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());
        dp.RCC
            .apb1enr
            .modify(|_, w| w.i2c1en().set_bit().pwren().set_bit());
        dp.RCC.cfgr3.modify(|_, w| w.i2c1sw().sysclk());
        dp.RCC.ahbenr.modify(|_, w| w.iopben().set_bit());

        rprintln!("Initializing peripherals");

        let mut rcc = dp
            .RCC
            .configure()
            .usbsrc(stm32f0xx_hal::rcc::USBClockSource::HSI48)
            .hsi48()
            .enable_crs(dp.CRS)
            .sysclk(48.mhz())
            .pclk(24.mhz())
            .freeze(&mut dp.FLASH);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let led = disable_interrupts(|cs| gpioa.pa5.into_push_pull_output(cs));

        let gpioc = dp.GPIOC.split(&mut rcc);
        let user_button = disable_interrupts(|cs| gpioc.pc13.into_floating_input(cs));

        dp.SYSCFG.exticr4.write(|w| w.exti13().pc13());

        // Set interrupt mask for all the above
        dp.EXTI.imr.write(|w| w.mr13().set_bit());

        // Set interrupt rising trigger
        dp.EXTI.ftsr.write(|w| w.tr13().set_bit());
        dp.EXTI.rtsr.write(|w| w.tr13().set_bit());

        rprintln!("Instantiating dp.EXTI...");
        let exti = dp.EXTI;
        rprintln!("Defining late resources...");

        let gpiob = dp.GPIOB.split(&mut rcc);

        let (scl, sda): (PB8<Alternate<AF1>>, PB9<Alternate<AF1>>) = disable_interrupts(|cs| {
            (
                gpiob.pb8.into_alternate_af1(cs),
                gpiob.pb9.into_alternate_af1(cs),
            )
        });

        const I2C_MODE_SMBUS_AUTOEND_WITH_PEC: u32 = (1 << 25) | (1 << 26);

        unsafe {
            dp.I2C1.icr.reset();

            dp.I2C1.cr2.modify(|_, w| w.autoend().set_bit());
            dp.I2C1.oar1.modify(|_, w| w.oa1en().clear_bit());
            dp.I2C1.oar2.modify(|_, w| w.oa2en().clear_bit());
            dp.I2C1.cr1.modify(|_, w| w.gcen().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.nostretch().clear_bit());
            dp.I2C1.cr1.modify(|_, w| w.pe().clear_bit());
            dp.I2C1
                .cr1
                .modify(|_, w| w.anfoff().set_bit().dnf().no_filter());
            dp.I2C1.cr1.modify(|_, w| w.pe().set_bit());
            dp.I2C1.oar1.write(|w| w.oa1().bits(0x33));
            dp.I2C1
                .cr1
                .modify(|r, w| w.bits(r.bits() | I2C_MODE_SMBUS_AUTOEND_WITH_PEC));
            //dp.I2C1.cr2.modify(|_, w| w.nack().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.pecen().set_bit());
            dp.I2C1.oar1.modify(|_, w| w.oa1en().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.sbc().set_bit());

            dp.I2C1.cr1.modify(|_, w| w.txie().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.rxie().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.addrie().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.nackie().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.stopie().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.tcie().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.errie().set_bit());
            dp.I2C1.cr2.modify(|_, w| w.nbytes().bits(0x1));
        }

        //let a: bool = dp.I2C1.isr.read().dir();

        let state = SMBusState::default();

        //I2c<I2C1, PB8<Alternate<AF1>>, PB9<Alternate<AF1>>>,
        init::LateResources {
            exti,
            user_button,
            led,
            i2c: dp.I2C1,
            transmissionState: state,
        }
    }

    #[idle(resources = [])]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
            // disable wfe for now (no probe-rs)
            //cortex_m::asm::wfe();
        }
    }

    #[task(binds = I2C1, resources = [i2c, user_button, led, transmissionState], priority = 1)]
    fn i2c1_interrupt(ctx: i2c1_interrupt::Context) {
        rprintln!("Something happened on SMBUS!");
        let isrReg = ctx.resources.i2c.isr.read().bits();
        let dataReg = ctx.resources.i2c.rxdr.read().bits();

        if ctx.resources.i2c.isr.read().addr().is_match_() {
            if ctx.resources.i2c.isr.read().dir().is_read() {
                ctx.resources.transmissionState.dir = SMBusDirection::SlaveToMaster;
                //trcount = 0;
                execute_smbus_command(ctx.resources.transmissionState);
                //ctx.resources.i2c.isr.txe().set_bit();
            }
        }
        /*
        if (isrReg & 1 << 3) == 1 << 3 {
            if (isrReg & 1 << 16) == 1 << 16 {

            }
        }
        */

        rprintln!("{}", ctx.resources.i2c.isr.read().bits());
        ctx.resources.i2c.icr.write(|w| w.addrcf().set_bit());
        ctx.resources.i2c.icr.reset();
        unsafe {
            ctx.resources.i2c.icr.write(|w| w.bits(0xFF));
        }
    }

    #[task(binds = EXTI4_15, resources = [exti, user_button, led])]
    fn exti_4_15_interrupt(ctx: exti_4_15_interrupt::Context) {
        match ctx.resources.exti.pr.read().bits() {
            0x2000 => {
                ctx.resources.exti.pr.write(|w| w.pif13().set_bit());
                if ctx.resources.user_button.is_high().unwrap_or_default() {
                    ctx.resources.led.set_high().unwrap();
                    rprintln!("PC13 triggered: high");
                } else {
                    ctx.resources.led.set_low().unwrap();
                    rprintln!("PC13 triggered: low");
                }
            }

            x => rprintln!("{}: Some other bits were pushed around on EXTI4_15 ;)", x),
        }
    }
};
