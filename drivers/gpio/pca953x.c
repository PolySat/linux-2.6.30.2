/*
 *  pca953x.c - 4/8/16 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <delay.h>
#include <errno.h>
  
#include <asm/gpio.h>
extern int at91_set_gpio_value(unsigned pin, int value); 

#define PCA953X_INPUT          0
#define PCA953X_OUTPUT         1
#define PCA953X_INVERT         2
#define PCA953X_DIRECTION      3

static const struct i2c_device_id pca953x_id[] = {
	{ "pca9534", 8, },
	{ "pca9535", 16, },
	{ "pca9536", 4, },
	{ "pca9537", 4, },
	{ "pca9538", 8, },
	{ "pca9539", 16, },
	{ "pca9554", 8, },
	{ "pca9555", 16, },
	{ "pca9557", 8, },

	{ "max7310", 8, },
	{ "pca6107", 8, },
	{ "tca6408", 8, },
	{ "tca6416", 16, },
	{ "pi4ioe5v9539", 16,} 
	/* NYET:  { "tca6424", 24, }, */
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca953x_id);

struct pca953x_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_direction;
	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	uint32_t reset_pin_number;  // if pi4ioe5v9539, consider reset pin ***** TO DO: **** consider macro for this?
};

static int pca953x_write_reg(struct pca953x_chip *chip, int reg, uint16_t val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	else 
		ret = i2c_smbus_write_word_data(chip->client, reg << 1, val);

	if (ret < 0) {
		// fail to write reg: retry 3 times max
		int attempts = 1; 
		while(ret < 0 && attempts < 3){
			if (chip->gpio_chip.ngpio <= 8)
				ret = i2c_smbus_write_byte_data(chip->client, reg, val);
			else 
				ret = i2c_smbus_write_word_data(chip->client, reg << 1, val);
			attempts++;
		}
		return ret;
	}

	return 0;
}

static int pca953x_read_reg(struct pca953x_chip *chip, int reg, uint16_t *val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_read_byte_data(chip->client, reg);
	else
		ret = i2c_smbus_read_word_data(chip->client, reg << 1);

	if (ret < 0) {
		int attempts = 1; 
		while(ret < 0 && attempts < 3){
			// retry 3 times max
			if (chip->gpio_chip.ngpio <= 8)
				ret = i2c_smbus_read_byte_data(chip->client, reg);
			else
				ret = i2c_smbus_read_word_data(chip->client, reg << 1);
			attempts++;
		}
		// dev_err(&chip->client->dev, "failed reading register\n"); 
		/* TO DO: move dev_error("failed reading reg") --> paretn function, only after:
		*		1) checking IF the chip is resettable(checking if pi4io) : in reset()
		*			1b) if resettable: resetting the chip if possible --> successful reset? depends on reset() return val...
		* 				(a) if successful reset():      retry reading registers --> fail ? dev_error("fail reading register")
		* 				(b) if NOT successful reset():  immediately return dev_error("failed reset")
		*			1c) if NOT: immediately return dev_error("failed reading register")
		*/
		return ret;
	}
	*val = (uint16_t)ret;
	return 0;
}

static int pca953x_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;
	chip = container_of(gc, struct pca953x_chip, gpio_chip);
	reg_val = chip->reg_direction | (1u << off);
	ret = pca953x_write_reg(chip, PCA953X_DIRECTION, reg_val); // returns < 0 if failed to write 3 times
	if (ret){
		// ret = pca953x_reset(chip, i2c_client CLIENT); // **** TO DO *****  WHERE TO GET CLIENT?
		// *** TO DO : handle difference between -EIO for I2C hang error, and failed to reset(at91 pin set fail: -EINVAL)
		// if outside: read wroite
		return ret;
	}
	chip->reg_direction = reg_val;
	return 0;
}

static int pca953x_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca953x_write_reg(chip, PCA953X_OUTPUT, reg_val);
	if (ret)
		// reset
		// ret = pca953x_reset(chip, client???) **** TO DO :  call RESET() function here?? *****
		return ret;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	ret = pca953x_write_reg(chip, PCA953X_DIRECTION, reg_val);
	if (ret)
		return ret;

	chip->reg_direction = reg_val;
	return 0;
}

static int pca953x_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	ret = pca953x_read_reg(chip, PCA953X_INPUT, &reg_val);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void pca953x_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca953x_write_reg(chip, PCA953X_OUTPUT, reg_val);
	if (ret){
		// failed to write 3 times, resetting IF resettable
		if(pca953x_chip->reset_pin_number)
		// int resetRet = pca953x_reset(chip, CLIENT); // *** TO DO: where to get i2c_client obj from? ****
		// if(not resettable or failed to write to reg): dev error give up 
		if(resetRet > 0)
			dev_err(&chip->client->dev, "reset not available\n");
		if(resetRet < 0)
			dev_err(&chip->client->dev, "failed writing register\n");
		return;
	}
	chip->reg_output = reg_val; 
}

/* Custom reset() function for pi4ioe5v9539 RESET pin: 
*	this fxn can be called by in case chip->reset_pin_number != 0 (assuming it exists)
*	Return Codes:
*	--> return 0: sucess
*	--> return -EOI: resettable; failed write to registers after chip reset
* 	--> return -EINVAL: fail to set RESET pin
*/
static int pca953x_reset(struct pca953x_chip *chip, struct i2c_client *client) {
	int ret = 0;
	// sleep for 25ns to allow the chip to reset
	ret = at91_set_gpio_value(chip->reset_pin_number, 1); 
	if(ret)
		return ret;
	ndelay(25); 
	ret = at91_set_gpio_value(chip->reset_pin_number, 0);
	if(ret)
		return ret; 
	// if anything for 191-193 fails, return -1
	// sleep for 1 ms to allow the chip to come back up after reset
	udelay(1);
	// load cache back to registers
	ret = pca953x_write_reg(chip, PCA953X_OUTPUT, chip->reg_output);
	if(ret < 0)
		return ret;
	ret = pca953x_write_reg(chip, PCA953X_DIRECTION, chip->reg_direction);
	if(ret < 0)
		return ret;
	ret = pca953x_write_reg(chip, PCA953X_INVERT, chip->reg_invert); 
	if(ret < 0)
		return ret;

}

static void pca953x_setup_gpio(struct pca953x_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pca953x_gpio_direction_input;
	gc->direction_output = pca953x_gpio_direction_output;
	gc->get = pca953x_gpio_get_value;
	gc->set = pca953x_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
}

static int __devinit pca953x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pca953x_platform_data *pdata;
	struct pca953x_chip *chip;
	int ret;

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pca953x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;
	chip->reset_pin_number = pdata->reset_pin_number;
	chip->gpio_start = pdata->gpio_base;
	chip->reg_invert = pdata->invert;

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pca953x_setup_gpio(chip, id->name);

	if(strcmp(id->name, "pi4ioe5v9539") != 0){
		// this is NOT a resettable gpio expander, hard set reset_pin_number = 0 (safety)
		chip->reset_pin_number = 0;
	}
	// simply need to test if chip->reset_pin_number != 0 to call reset() function

	ret = pca953x_read_reg(chip, PCA953X_OUTPUT, &chip->reg_output);
	if (ret) {
      printk("*** read output failed pca\n\r");
		goto out_failed;
   }

	ret = pca953x_read_reg(chip, PCA953X_DIRECTION, &chip->reg_direction);
	if (ret) {
      printk("*** read dir failed pca\n\r");
		goto out_failed;
   }

	/* set platform specific polarity inversion */
	ret = pca953x_write_reg(chip, PCA953X_INVERT, pdata->invert);
	if (ret) {
      printk("*** write invert failed pca\n\r");
		goto out_failed;
   }

	ret = gpiochip_add(&chip->gpio_chip);
	if (ret) {
      printk("*** gpio add failed pca\n\r");
		goto out_failed;
   }

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);
	return 0;

	out_failed:
    	printk("*** out failed pca953x \n\r");
		kfree(chip);
		return ret;
}

static int pca953x_remove(struct i2c_client *client)
{
	struct pca953x_platform_data *pdata = client->dev.platform_data;
	struct pca953x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	kfree(chip);
	return 0;
}

static struct i2c_driver pca953x_driver = {
	.driver = {
		.name	= "pca953x",
	},
	.probe		= pca953x_probe,
	.remove		= pca953x_remove,
	.id_table	= pca953x_id,
};

static int __init pca953x_init(void)
{
	return i2c_add_driver(&pca953x_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(pca953x_init);

static void __exit pca953x_exit(void)
{
	i2c_del_driver(&pca953x_driver);
}
module_exit(pca953x_exit);

MODULE_AUTHOR("eric miao <eric.miao@marvell.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCA953x");
MODULE_LICENSE("GPL");
