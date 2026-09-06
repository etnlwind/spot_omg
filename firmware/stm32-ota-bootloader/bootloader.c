#include "stm32f446xx.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_ADDRESS          0x08010000UL
#define APP_LIMIT_ADDRESS    0x08060000UL
#define METADATA_ADDRESS     0x0800C000UL
#define SRAM_START           0x20000000UL
#define SRAM_END             0x20020000UL
#define REQUEST_ADDRESS      0x2001FFF0UL
#define REQUEST_MAGIC        0x53504F54UL
#define METADATA_MAGIC       0x53504657UL
#define PROTOCOL_VERSION     1UL
#define ACK_INTERVAL         1024UL

typedef void (*EntryPoint)(void);

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc32;
    uint32_t version;
} ImageMetadata;

void SystemInit(void)
{
    /* Reset clock is the 16 MHz HSI used by uart_init(). */
}

void _init(void) {}
void _fini(void) {}

static void uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;

    GPIOC->MODER = (GPIOC->MODER & ~((3UL << 20U) | (3UL << 22U))) |
                   (2UL << 20U) | (2UL << 22U);
    GPIOC->OSPEEDR |= (3UL << 20U) | (3UL << 22U);
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~((0xFUL << 8U) | (0xFUL << 12U))) |
                    (7UL << 8U) | (7UL << 12U);
    USART3->BRR = 139U;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_putc(uint8_t value)
{
    while ((USART3->SR & USART_SR_TXE) == 0U) {}
    USART3->DR = value;
}

static uint8_t uart_getc(void)
{
    while ((USART3->SR & USART_SR_RXNE) == 0U) {
        if ((USART3->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U) {
            volatile uint32_t discarded = USART3->DR;
            (void)discarded;
        }
    }
    return (uint8_t)USART3->DR;
}

static void uart_text(const char *text)
{
    while (*text != '\0') {
        uart_putc((uint8_t)*text++);
    }
}

static void uart_u32(uint32_t value)
{
    char digits[10];
    size_t count = 0U;
    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count != 0U) uart_putc((uint8_t)digits[--count]);
}

static uint32_t crc32_update(uint32_t crc, uint8_t value)
{
    crc ^= value;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
    }
    return crc;
}

static uint32_t image_crc32(uint32_t address, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t offset = 0U; offset < size; ++offset) {
        crc = crc32_update(crc, *(const uint8_t *)(address + offset));
    }
    return crc ^ 0xFFFFFFFFUL;
}

static bool vector_valid(void)
{
    const uint32_t stack = *(const uint32_t *)APP_ADDRESS;
    const uint32_t reset = *(const uint32_t *)(APP_ADDRESS + 4U);
    return stack >= SRAM_START && stack <= SRAM_END &&
           (reset & 1U) != 0U && reset >= APP_ADDRESS && reset < APP_LIMIT_ADDRESS;
}

static bool installed_image_valid(void)
{
    const ImageMetadata *metadata = (const ImageMetadata *)METADATA_ADDRESS;
    return metadata->magic == METADATA_MAGIC &&
           metadata->version == PROTOCOL_VERSION &&
           metadata->size >= 8U &&
           metadata->size <= APP_LIMIT_ADDRESS - APP_ADDRESS &&
           vector_valid() &&
           image_crc32(APP_ADDRESS, metadata->size) == metadata->crc32;
}

static void jump_to_app(void)
{
    const uint32_t stack = *(const uint32_t *)APP_ADDRESS;
    const uint32_t reset = *(const uint32_t *)(APP_ADDRESS + 4U);
    USART3->CR1 = 0U;
    __disable_irq();
    SCB->VTOR = APP_ADDRESS;
    __set_MSP(stack);
    /* Reset_Handler expects the reset-state PRIMASK (interrupts enabled).
     * Leaving PRIMASK set prevents the application's SysTick ISR from
     * advancing HAL_GetTick(), so its first HAL_Delay() never returns. */
    __enable_irq();
    ((EntryPoint)reset)();
    for (;;) {}
}

static bool flash_wait(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0U) {}
    return (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                         FLASH_SR_PGPERR | FLASH_SR_PGSERR)) == 0U;
}

static void flash_clear_status(void)
{
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                FLASH_SR_PGPERR | FLASH_SR_PGSERR;
}

static void flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
        FLASH->KEYR = 0x45670123UL;
        FLASH->KEYR = 0xCDEF89ABUL;
    }
}

static bool flash_erase_sector(uint32_t sector)
{
    flash_clear_status();
    FLASH->CR = (FLASH->CR & ~(FLASH_CR_SNB | FLASH_CR_PSIZE)) |
                FLASH_CR_SER | FLASH_CR_PSIZE_1 |
                (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    const bool ok = flash_wait();
    FLASH->CR &= ~FLASH_CR_SER;
    return ok;
}

static bool flash_program_word(uint32_t address, uint32_t value)
{
    flash_clear_status();
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE) |
                FLASH_CR_PG | FLASH_CR_PSIZE_1;
    *(volatile uint32_t *)address = value;
    const bool ok = flash_wait() && *(const uint32_t *)address == value;
    FLASH->CR &= ~FLASH_CR_PG;
    return ok;
}

static bool read_header(uint32_t *size, uint32_t *crc)
{
    char line[64];
    size_t length = 0U;
    while (length + 1U < sizeof(line)) {
        const char value = (char)uart_getc();
        if (value == '\n') break;
        if (value != '\r') line[length++] = value;
    }
    line[length] = '\0';

    const char prefix[] = "SPOTFW 1 ";
    size_t index = 0U;
    while (prefix[index] != '\0') {
        if (line[index] != prefix[index]) return false;
        ++index;
    }
    uint32_t parsed_size = 0U;
    bool digit = false;
    while (line[index] >= '0' && line[index] <= '9') {
        digit = true;
        parsed_size = parsed_size * 10U + (uint32_t)(line[index++] - '0');
    }
    if (!digit || line[index++] != ' ') return false;
    uint32_t parsed_crc = 0U;
    uint8_t hex_digits = 0U;
    while (line[index] != '\0') {
        const char c = line[index++];
        uint8_t nibble;
        if (c >= '0' && c <= '9') nibble = (uint8_t)(c - '0');
        else if (c >= 'a' && c <= 'f') nibble = (uint8_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') nibble = (uint8_t)(c - 'A' + 10);
        else return false;
        if (++hex_digits > 8U) return false;
        parsed_crc = (parsed_crc << 4U) | nibble;
    }
    if (hex_digits != 8U || parsed_size < 8U ||
        parsed_size > APP_LIMIT_ADDRESS - APP_ADDRESS) return false;
    *size = parsed_size;
    *crc = parsed_crc;
    return true;
}

static bool receive_image(uint32_t size, uint32_t expected_crc)
{
    flash_unlock();
    if (!flash_erase_sector(3U) || !flash_erase_sector(4U) ||
        !flash_erase_sector(5U) || !flash_erase_sector(6U)) return false;
    uart_text("SPOTBOOT READY\n");

    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t address = APP_ADDRESS;
    uint32_t word = 0xFFFFFFFFUL;
    uint8_t word_bytes = 0U;
    for (uint32_t received = 0U; received < size; ++received) {
        const uint8_t value = uart_getc();
        crc = crc32_update(crc, value);
        word = (word & ~(0xFFUL << (8U * word_bytes))) |
               ((uint32_t)value << (8U * word_bytes));
        if (++word_bytes == 4U) {
            if (!flash_program_word(address, word)) return false;
            address += 4U;
            word = 0xFFFFFFFFUL;
            word_bytes = 0U;
        }
        if ((received + 1U) % ACK_INTERVAL == 0U || received + 1U == size) {
            uart_text("SPOTBOOT ACK ");
            uart_u32(received + 1U);
            uart_putc('\n');
        }
    }
    if (word_bytes != 0U && !flash_program_word(address, word)) return false;
    crc ^= 0xFFFFFFFFUL;
    if (crc != expected_crc || !vector_valid() ||
        image_crc32(APP_ADDRESS, size) != expected_crc) return false;

    if (!flash_program_word(METADATA_ADDRESS + 4U, size) ||
        !flash_program_word(METADATA_ADDRESS + 8U, expected_crc) ||
        !flash_program_word(METADATA_ADDRESS + 12U, PROTOCOL_VERSION) ||
        !flash_program_word(METADATA_ADDRESS, METADATA_MAGIC)) return false;
    FLASH->CR |= FLASH_CR_LOCK;
    return true;
}

int main(void)
{
    volatile uint32_t *request = (volatile uint32_t *)REQUEST_ADDRESS;
    const bool requested = *request == REQUEST_MAGIC;
    *request = 0U;
    if (!requested && installed_image_valid()) jump_to_app();

    uart_init();
    uart_text("SPOTBOOT 1\n");
    for (;;) {
        uint32_t size = 0U;
        uint32_t crc = 0U;
        if (!read_header(&size, &crc)) {
            uart_text("SPOTBOOT ERROR header\n");
            uart_text("SPOTBOOT 1\n");
            continue;
        }
        if (receive_image(size, crc)) {
            uart_text("SPOTBOOT OK\n");
            for (volatile uint32_t delay = 0U; delay < 1000000U; ++delay) {}
            NVIC_SystemReset();
        }
        uart_text("SPOTBOOT ERROR flash\n");
    }
}
