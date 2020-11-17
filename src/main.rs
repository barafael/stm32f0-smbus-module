#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

use stm32f0xx_hal::{
    gpio::gpioa::{PA0, PA1, PA15, PA2, PA3, PA4, PA5, PA6, PA7},
    gpio::gpiob::{PB1, PB3, PB4},
    gpio::{Input, Output, PullUp, PushPull},
    pac,
    prelude::*,
};

use cortex_m::interrupt::free as disable_interrupts;

#[app(device = stm32f0xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        exti: pac::EXTI,
        usr_led: PB1<Output<PushPull>>,
        button3: PA15<Input<PullUp>>,
        button4: PB4<Input<PullUp>>,
        button5: PB3<Input<PullUp>>,
        tb_left: PA4<Input<PullUp>>,
        tb_up: PA5<Input<PullUp>>,
        tb_right: PA6<Input<PullUp>>,
        tb_down: PA7<Input<PullUp>>,
        bbled_red: PA0<Output<PushPull>>,
        bbled_grn: PA1<Output<PushPull>>,
        bbled_blu: PA2<Output<PushPull>>,
        bbled_wht: PA3<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        // RTT handler
        rtt_init_print!();

        // Alias peripherals
        let mut dp: pac::Peripherals = ctx.device;

        // Power on bbled dance
        //bbled_red.toggle().ok();

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

        // Set up GPIO registers for USR LED and Buttons
        let gpiob = dp.GPIOB.split(&mut rcc);
        let (usr_led, button4, button5) = disable_interrupts(|cs| {
            (
                gpiob.pb1.into_push_pull_output(cs),
                gpiob.pb4.into_pull_up_input(cs),
                gpiob.pb3.into_pull_up_input(cs),
            )
        });

        // LEDs and USB
        let gpioa = dp.GPIOA.split(&mut rcc);
        let (
            bbled_red,
            bbled_grn,
            bbled_blu,
            bbled_wht,
            tb_left,
            tb_up,
            tb_right,
            tb_down,
            button3,
        ) = disable_interrupts(|cs| {
            (
                gpioa.pa0.into_push_pull_output(cs),
                gpioa.pa1.into_push_pull_output(cs),
                gpioa.pa2.into_push_pull_output(cs),
                gpioa.pa3.into_push_pull_output(cs),
                gpioa.pa4.into_pull_up_input(cs),
                gpioa.pa5.into_pull_up_input(cs),
                gpioa.pa6.into_pull_up_input(cs),
                gpioa.pa7.into_pull_up_input(cs),
                gpioa.pa15.into_pull_up_input(cs),
            )
        });

        // Enable external interrupt for 3 aux buttons...
        dp.SYSCFG.exticr1.write(|w| w.exti3().pb3());
        // dp.SYSCFG.exticr2.write(|w| { w.exti4().pb4() }); // Disable spare button in favor of tb_left
        dp.SYSCFG.exticr4.write(|w| w.exti15().pa15());
        //... and for pulses on trackball
        dp.SYSCFG.exticr2.write(|w| w.exti4().pa4());
        dp.SYSCFG.exticr2.write(|w| w.exti5().pa5());
        dp.SYSCFG.exticr2.write(|w| w.exti6().pa6());
        dp.SYSCFG.exticr2.write(|w| w.exti7().pa7());

        // Set interrupt mask for all the above
        dp.EXTI.imr.write(|w| {
            w.mr3().set_bit();
            w.mr4().set_bit();
            w.mr5().set_bit();
            w.mr6().set_bit();
            w.mr7().set_bit();
            w.mr15().set_bit()
        });

        // Set interrupt rising trigger
        dp.EXTI.rtsr.write(|w| {
            w.tr3().set_bit();
            w.tr4().set_bit();
            w.tr5().set_bit();
            w.tr6().set_bit();
            w.tr7().set_bit();
            w.tr15().set_bit()
        });

        rprintln!("Instantiating dp.EXTI...");
        let exti = dp.EXTI;
        rprintln!("Defining late resources...");

        init::LateResources {
            exti,
            usr_led,
            button3,
            button4,
            button5,
            tb_left,
            tb_up,
            tb_right,
            tb_down,
            bbled_red,
            bbled_grn,
            bbled_blu,
            bbled_wht,
        }
    }

    #[idle(resources = [])]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = EXTI2_3, resources = [exti])]
    fn exti2_3_interrupt(ctx: exti2_3_interrupt::Context) {
        rprintln!("Interrupts happening on EXTI2_3");

        match ctx.resources.exti.pr.read().bits() {
            0x8 => {
                rprintln!("PB3 triggered");
                ctx.resources.exti.pr.write(|w| w.pif3().set_bit()); // Clear interrupt
            }

            _ => rprintln!("Some other bits were pushed around on EXTI2_3 ;)"),
        }
    }

    #[task(binds = EXTI4_15, resources = [exti, usr_led, bbled_red, bbled_grn, bbled_wht, bbled_blu])]
    fn exti_4_15_interrupt(ctx: exti_4_15_interrupt::Context) {
        rprintln!("Interrupts happening on EXTI for PA15...");

        let usr_led = ctx.resources.usr_led;

        match ctx.resources.exti.pr.read().bits() {
            0x8000 => {
                rprintln!("PA15 triggered");
                ctx.resources.exti.pr.write(|w| w.pif15().set_bit()); // Clear interrupt
                usr_led.toggle().ok();
            }
            0x10 => {
                rprintln!("tb_left triggered!");
                ctx.resources.exti.pr.write(|w| w.pif4().set_bit());
                usr_led.toggle().ok();
            }
            0x20 => {
                rprintln!("tb_up triggered!");
                ctx.resources.exti.pr.write(|w| w.pif5().set_bit());
                usr_led.toggle().ok();
            }
            0x40 => {
                rprintln!("tb_right triggered!");
                ctx.resources.exti.pr.write(|w| w.pif6().set_bit());
                usr_led.toggle().ok();
            }
            0x80 => {
                rprintln!("tb_down triggered!");
                ctx.resources.exti.pr.write(|w| w.pif7().set_bit());
                usr_led.toggle().ok();
            }

            _ => rprintln!("Some other bits were pushed around on EXTI4_15 ;)"),
        }
    }
};
