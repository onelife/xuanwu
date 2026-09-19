/*
 * Floating-point acceptance firmware for the STM32F411 (Cortex-M4F).
 *
 * It exists so the FPU support can be tested end to end instead of only with
 * hand-written instruction snippets:
 *
 *   1. single-precision arithmetic, compiled to VFP instructions;
 *   2. the floating-point extension registers report the expected features;
 *   3. a SysTick handler deliberately clobbers S0-S3 while main() is holding
 *      values in exactly those registers.  The values must survive, which only
 *      happens if the exception stacked and restored the floating-point frame.
 *
 * Results are published in the zero-initialised `result` array, which the test
 * reads straight out of RAM.  Everything is assigned at run time so the image
 * needs no C startup code and no initialised data.
 *
 * Build with build.sh; the toolchain is arm-none-eabi-gcc.
 */

#include <stdint.h>

#define MAGIC 0x58575546u /* "XWUF" */

volatile uint32_t result[16];

#define RESULT_MAGIC 0
#define RESULT_SUM 1
#define RESULT_PRODUCT 2
#define RESULT_CONVERT 3
#define RESULT_MVFR0 4
#define RESULT_MVFR1 5
#define RESULT_FPCCR 6
#define RESULT_IRQ_COUNT 7
#define RESULT_A 8
#define RESULT_B 9
#define RESULT_C 10
#define RESULT_D 11
#define RESULT_PARKED 12

#define FPU_BASE 0xE000EF30u
#define FPU_FPCCR (*(volatile uint32_t *)(FPU_BASE + 0x04))
#define FPU_MVFR0 (*(volatile uint32_t *)(FPU_BASE + 0x10))
#define FPU_MVFR1 (*(volatile uint32_t *)(FPU_BASE + 0x14))

#define SYSTICK_CSR (*(volatile uint32_t *)0xE000E010u)
#define SYSTICK_RVR (*(volatile uint32_t *)0xE000E014u)
#define SYSTICK_CVR (*(volatile uint32_t *)0xE000E018u)
#define SCB_CPACR (*(volatile uint32_t *)0xE000ED88u)

volatile uint32_t irq_count;
volatile uint32_t parked;

/* Set before the interrupt loop, clobbered by the handler. */
static float keep_a, keep_b, keep_c, keep_d;
static float junk_a, junk_b;

void SysTick_Handler(void) {
    /* Overwrite single-precision state on purpose. */
    __asm__ volatile(
        "vldr s0, [%0]\n"
        "vldr s1, [%0]\n"
        "vldr s2, [%1]\n"
        "vldr s3, [%1]\n"
        "vadd.f32 s4, s0, s1\n"
        "vmov.f32 s5, s2\n"
        :
        : "r"(&junk_a), "r"(&junk_b)
        : "s0", "s1", "s2", "s3", "s4", "s5");
    irq_count++;
}

__attribute__((used)) static float add(float x, float y) { return x + y; }

/* Hold four values in S0-S3 across the interrupts, then store them out. */
__attribute__((used)) static void hold_across_interrupts(volatile uint32_t *out, volatile uint32_t *counter) {
    __asm__ volatile(
        "vldr s0, [%0]\n"
        "vldr s1, [%1]\n"
        "vldr s2, [%2]\n"
        "vldr s3, [%3]\n"
        "1:\n"
        "ldr r3, [%8]\n"
        "cmp r3, #3\n"
        "blt 1b\n"
        "vstr s0, [%4]\n"
        "vstr s1, [%5]\n"
        "vstr s2, [%6]\n"
        "vstr s3, [%7]\n"
        :
        : "r"(&keep_a), "r"(&keep_b), "r"(&keep_c), "r"(&keep_d),
          "r"(&out[RESULT_A]), "r"(&out[RESULT_B]), "r"(&out[RESULT_C]), "r"(&out[RESULT_D]),
          "r"(counter)
        : "r3", "s0", "s1", "s2", "s3", "memory", "cc");
}

int main(void) {
    /* Real firmware enables the FPU before using it. */
    SCB_CPACR |= (0xFu << 20);

    keep_a = 1.5f;
    keep_b = 2.25f;
    keep_c = -3.75f;
    keep_d = 100.0f;
    junk_a = 9.0f;
    junk_b = 8.0f;

    result[RESULT_MAGIC] = MAGIC;
    result[RESULT_SUM] = (uint32_t)(add(1.5f, 2.25f) == 3.75f);
    result[RESULT_PRODUCT] = (uint32_t)((1.5f * 2.0f) == 3.0f);
    result[RESULT_CONVERT] = (uint32_t)(((float)(int)3 + 0.5f) == 3.5f);

    result[RESULT_MVFR0] = FPU_MVFR0;
    result[RESULT_MVFR1] = FPU_MVFR1;
    result[RESULT_FPCCR] = FPU_FPCCR;

    irq_count = 0;
    /*
     * The simulator advances SysTick one instruction at a time (one cycle per
     * instruction unless the chip description says otherwise), so RVR is chosen
     * large enough that the main line makes progress between interrupts: a tiny
     * RVR re-pends the interrupt on nearly every instruction and the core
     * tail-chains into the handler forever, never returning to thread mode.
     * 4096 cycles per period gives three interrupts well inside 20k
     * instructions.
     */
    SYSTICK_RVR = 0xFFF;
    SYSTICK_CVR = 0;
    SYSTICK_CSR = 0x7; /* enable | tickint | processor clock */

    hold_across_interrupts(result, &irq_count);

    SYSTICK_CSR = 0;
    result[RESULT_IRQ_COUNT] = irq_count;

    parked = 1;
    for (;;) {
        __asm__ volatile("nop");
    }
}

/* Minimal vector table: reset and SysTick are all this firmware needs. */
extern uint32_t _estack;

__attribute__((section(".vectors"), used)) void (*const vectors[])(void) = {
    (void (*)(void)) & _estack,
    (void (*)(void)) main,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    SysTick_Handler, /* exception 15 -> offset 0x3C */
};
