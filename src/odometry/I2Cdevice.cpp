#include "odometry/I2Cdevice.h"
#include "clock.h"
#include "config.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>

const int i2c = I2C3;

void setupDeviceI2C(){
    rcc_periph_clken rcc_i2c = RCC_I2C3;
	rcc_periph_clock_enable(rcc_i2c);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	//rcc_periph_clock_enable(RCC_GPIOH);
	//rcc_set_i2c_clock_hsi(i2c); //TODO might need to enable this for HSI clock

	i2c_peripheral_disable(i2c);

	gpio_mode_setup(port_OdoClk, GPIO_MODE_AF, GPIO_PUPD_NONE, pin_OdoClk);
	gpio_mode_setup(port_OdoSda, GPIO_MODE_AF, GPIO_PUPD_NONE, pin_OdoSda);

	// it's important to set the pins to open drain
	gpio_set_output_options(port_OdoClk, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, pin_OdoClk);
	gpio_set_output_options(port_OdoSda, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, pin_OdoSda);
	gpio_set_af(port_OdoClk, GPIO_AF4, pin_OdoClk);
	gpio_set_af(port_OdoSda, GPIO_AF4, pin_OdoSda);
	
	/* HSI is at 82Mhz */
	i2c_set_speed(i2c, i2c_speed_sm_100k, 42);
	//addressing mode
	//i2c_set_7bit_addr_mode(i2c);
	i2c_set_standard_mode(i2c);
	i2c_peripheral_enable(i2c);
}

#include "libopencm3/stm32/i2c.h"
#include "libopencm3/stm32/rcc.h"
#include <stdint.h>
#include <stddef.h>


#define I2C_TIMEOUT_MS 5

// External millisecond counter (SysTick)
extern volatile uint32_t systicks;

// Reset the I2C peripheral and bus
static void i2c_bus_reset()
{
    i2c_peripheral_disable(i2c);

    // Clear CR1/CR2
    I2C_CR1(i2c) = 0;
    I2C_CR2(i2c) = 0;

    i2c_peripheral_enable(i2c);

    // Generate STOP to release bus
    i2c_send_stop(i2c);
}

// --- Safe 7-bit write ---
int i2c_write7_safe(uint8_t addr, const uint8_t *data, size_t n)
{
    uint32_t start = systicks;

    // Wait until BUSY cleared
    while (I2C_SR2(i2c) & I2C_SR2_BUSY) {
        if ((systicks - start) >= I2C_TIMEOUT_MS) {
            i2c_bus_reset();
            return -1;
        }
    }

    i2c_send_start(i2c);

    // Wait for SB, MSL, BUSY
    start = systicks;
    while (!( (I2C_SR1(i2c) & I2C_SR1_SB) &&
              (I2C_SR2(i2c) & I2C_SR2_MSL) &&
              (I2C_SR2(i2c) & I2C_SR2_BUSY) )) {
        if ((systicks - start) >= I2C_TIMEOUT_MS) {
            i2c_bus_reset();
            return -1;
        }
    }

    i2c_send_7bit_address(i2c, addr, I2C_WRITE);

    // Wait for ADDR flag
    start = systicks;
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
        if ((systicks - start) >= I2C_TIMEOUT_MS) {
            i2c_bus_reset();
            return -1;
        }
    }
    (void)I2C_SR2(i2c); // clear ADDR

    for (size_t i = 0; i < n; i++) {
        i2c_send_data(i2c, data[i]);
        start = systicks;
        while (!(I2C_SR1(i2c) & I2C_SR1_BTF)) {
            if ((systicks - start) >= I2C_TIMEOUT_MS) {
                i2c_bus_reset();
                return -1;
            }
        }
    }

    i2c_send_stop(i2c);

	// wait for 50 us
	for(uint32_t i = 0; i < 1000; i++){
		asm("nop");
	}

    return 0;
}

// --- Safe 7-bit read ---
int i2c_read7_safe(uint8_t addr, uint8_t *res, size_t n)
{
    i2c_send_start(i2c);
    i2c_enable_ack(i2c);

    uint32_t start = systicks;
    // Wait for SB, MSL, BUSY
    while (!( (I2C_SR1(i2c) & I2C_SR1_SB) &&
              (I2C_SR2(i2c) & I2C_SR2_MSL) &&
              (I2C_SR2(i2c) & I2C_SR2_BUSY) )) {
        if ((systicks - start) >= I2C_TIMEOUT_MS) {
            i2c_bus_reset();
            return -1;
        }
    }

    i2c_send_7bit_address(i2c, addr, I2C_READ);

    // Wait for ADDR
    start = systicks;
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
        if ((systicks - start) >= I2C_TIMEOUT_MS) {
            i2c_bus_reset();
            return -1;
        }
    }
    (void)I2C_SR2(i2c); // clear ADDR

    for (size_t i = 0; i < n; i++) {
        if (i == n - 1) i2c_disable_ack(i2c);

        start = systicks;
        while (!(I2C_SR1(i2c) & I2C_SR1_RxNE)) {
            if ((systicks - start) >= I2C_TIMEOUT_MS) {
                i2c_bus_reset();
                return -1;
            }
        }

        res[i] = i2c_get_data(i2c);
    }

    i2c_send_stop(i2c);
    return 0;
}


// --- safe transfer ---
int i2c_transfer7_safe(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    if (wn && i2c_write7_safe(addr, w, wn) < 0)
        return -1;

    if (rn)
        return i2c_read7_safe(addr, r, rn);

    return 0;
}





I2CDevice::I2CDevice(uint8_t address)
{
    setAddress(address);
    setRetries(3);
}

int I2CDevice::ping()
{
	// ping the device
	return i2c_transfer7_safe(_address, nullptr, 0, nullptr, 0);
}

int I2CDevice::readRegisters(uint8_t reg, uint8_t *data, uint8_t size, int &bytesRead)
{
    //i2c_transfer7(i2c, _address, &reg, 1, data, size);
	int ret = i2c_transfer7_safe(_address, &reg, 1, data, size);
	bytesRead = size;
    return ret;
}

int I2CDevice::readRegister(uint8_t reg, uint8_t &data)
{
    int bytesRead;
    return readRegisters(reg, &data, 1, bytesRead);
}

int I2CDevice::writeRegisters(uint8_t reg, uint8_t *data, uint8_t size)
{
    //i2c_transfer7(i2c, _address, &reg, 1, data, size);
	uint8_t buf[1 + size];
	buf[0] = reg;
	for (uint8_t i = 0; i < size; i++)
		buf[i + 1] = data[i];
	return i2c_transfer7_safe(_address, buf, 1 + size, nullptr, 0);
}

int I2CDevice::writeRegister(uint8_t reg, uint8_t data)
{
    return writeRegisters(reg, &data, 1);
}

void I2CDevice::setAddress(uint8_t address)
{
    _address = address;
}