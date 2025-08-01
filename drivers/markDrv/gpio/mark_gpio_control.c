#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/printk.h>


struct gpio_gpio {
	int gpio_num;
	int val;
};

#define TEST true
#define DEBUG(x...) if(TEST){printk(x);}

struct power_en_gpio {
	int sleep_flag;
	struct wake_lock rp_wake_lock;
	struct gpio_gpio gpio_spken;
	struct gpio_gpio gpio_pwen;
	struct gpio_gpio gpio_sdemmc;
	struct gpio_gpio gpio_fan;
};






static int power_en_probe(struct platform_device* pdev) {


	int b;
	int ret = 0;
	struct device_node* np = pdev->dev.of_node;//DTB转成device_node 树状结构
	struct power_en_gpio* data;
	data = devm_kzalloc(&pdev->dev, sizeof(struct power_en_gpio), GFP_KERNEL);   //内核内存分配函数 卸载（空载）时，内存会被自动释放,GFP_KERNEL


	if (!data) {
		dev_err(&pdev->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}
	memset(data, 0, sizeof(struct power_en_gpio));//void *memset(void *buffer, int c, int count)  c:赋给buffer的值 count buffer的长度


/******** gpio_sdemmc  *********/
	data->gpio_sdemmc.gpio_num = of_get_named_gpio_flags(np, "led_work", 0, NULL);//从设备树中读取sdemmc 的 GPIO 配置编号和标志

	if (!gpio_is_valid(data->gpio_sdemmc.gpio_num))
	{
		data->gpio_sdemmc.gpio_num = -1; //gpio_is_valid 判断该 GPIO 编号是否有效
		DEBUG("jerry sdemmc gpio is error:%d\n", data->gpio_sdemmc.gpio_num);
	}
	platform_set_drvdata(pdev, data);//可以将ndev保存成平台总线设备的私有数据	

	//判断GPIO申请成功
	if (data->gpio_sdemmc.gpio_num != -1) {
		ret = gpio_request(data->gpio_sdemmc.gpio_num, "led_work");
		DEBUG("jerry think sdemmc_pin request is:%d\n", ret);
		if (ret < 0) {
			DEBUG("data->sdemmc request error\n");
			//	return ret;
		}
		else {

			gpio_direction_output(data->gpio_sdemmc.gpio_num, 1); //设为输出
			gpio_set_value(data->gpio_sdemmc.gpio_num, 1);
	//		int b;
			b = data->gpio_sdemmc.gpio_num;
			DEBUG("jerry think the sdemmc value is:%d\n", b);
		}
	}



	/* gpio_fan */
	data->gpio_fan.gpio_num = of_get_named_gpio_flags(np, "gpio_fan", 0, NULL);//从设备树中读取sdemmc 的 GPIO 配置编号和标志

	if (!gpio_is_valid(data->gpio_fan.gpio_num))
	{
		data->gpio_fan.gpio_num = -1; //gpio_is_valid 判断该 GPIO 编号是否有效
		DEBUG("jerry fan_gpio gpio is error:%d\n", data->gpio_fan.gpio_num);
	}
	platform_set_drvdata(pdev, data);//可以将ndev保存成平台总线设备的私有数据	

	//判断GPIO申请成功
	if (data->gpio_fan.gpio_num != -1) {
		ret = gpio_request(data->gpio_fan.gpio_num, "gpio_fan");
		DEBUG("jerry think fan_pin request is:%d\n", ret);
		if (ret < 0) {
			DEBUG("data->fan request error\n");
			//	return ret;
		}
		else {

			gpio_direction_output(data->gpio_fan.gpio_num, 1); //设为输出
			gpio_set_value(data->gpio_fan.gpio_num, 1);
	//		int b;
			b = data->gpio_fan.gpio_num;
			DEBUG("jerry think the fan_gpio value is:%d\n", b);
		}
	}





	/* gpio_pwen*/
	data->gpio_pwen.gpio_num = of_get_named_gpio_flags(np, "gpio_pwen", 0, NULL);//从设备树中读取sdemmc 的 GPIO 配置编号和标志

	if (!gpio_is_valid(data->gpio_pwen.gpio_num))
	{
		data->gpio_pwen.gpio_num = -1; //gpio_is_valid 判断该 GPIO 编号是否有效
		DEBUG("jerry pwn gpio is error:%d\n", data->gpio_pwen.gpio_num);
	}
	platform_set_drvdata(pdev, data);//可以将ndev保存成平台总线设备的私有数据	

	//判断GPIO申请成功
	if (data->gpio_pwen.gpio_num != -1) {
		ret = gpio_request(data->gpio_pwen.gpio_num, "gpio_pwen");
		DEBUG("jerry think pwen_pin request is:%d\n", ret);
		if (ret < 0) {
			DEBUG("data->pwen request error\n");
			//	return ret;
		}
		else {


			gpio_direction_output(data->gpio_pwen.gpio_num, 0); //设为输出
			gpio_set_value(data->gpio_pwen.gpio_num, 0);
			gpio_free(data->gpio_pwen.gpio_num);		//relase gpio
	//		int b;
			b = data->gpio_pwen.gpio_num;
			DEBUG("jerry think the pwen value is:%d\n", b);
		}
	}

	/* gpio_spken*/


	data->gpio_spken.gpio_num = of_get_named_gpio_flags(np, "gpio_spken", 0, NULL);//从设备树中读取sdemmc 的 GPIO 配置编号和标志

	if (!gpio_is_valid(data->gpio_spken.gpio_num))
	{
		data->gpio_spken.gpio_num = -1; //gpio_is_valid 判断该 GPIO 编号是否有效
		DEBUG("jerry spk_gpio gpio is error:%d\n", data->gpio_spken.gpio_num);
	}
	platform_set_drvdata(pdev, data);//可以将ndev保存成平台总线设备的私有数据	

	//判断GPIO申请成功
	if (data->gpio_spken.gpio_num != -1) {
		ret = gpio_request(data->gpio_spken.gpio_num, "gpio_spken");
		DEBUG("jerry think spk_gpio_pin request is:%d\n", ret);
		if (ret < 0) {
			DEBUG("data->spken request error\n");
			//	return ret;
		}
		else {

			gpio_direction_output(data->gpio_spken.gpio_num, 1); //设为输出
			gpio_set_value(data->gpio_spken.gpio_num, 1);
	//		int b;
			b = data->gpio_spken.gpio_num;
			DEBUG("jerry think the spk_gpio value is:%d\n", b);
		}
	}


	if (data->sleep_flag != 0) {
		wake_lock_init(&data->rp_wake_lock, WAKE_LOCK_SUSPEND, "no_deep_sleep");//WAKE_LOCK_SUSPEND, // 阻止进入深度休眠模式 
		wake_lock(&data->rp_wake_lock);
	}

	printk("jerry probe is successl\n");

	return 0;

}


static int power_en_remove(struct platform_device* pdev) {
	struct power_en_gpio* data = platform_get_drvdata(pdev);
	if (data->gpio_sdemmc.gpio_num != -1) {
		gpio_direction_output(data->gpio_sdemmc.gpio_num, 0);
		gpio_set_value(data->gpio_sdemmc.gpio_num, 0);
	}


	if (data->gpio_fan.gpio_num != -1) {
		gpio_direction_output(data->gpio_fan.gpio_num, 0);
		gpio_set_value(data->gpio_fan.gpio_num, 0);
	}

	if (data->gpio_pwen.gpio_num != -1) {
		gpio_direction_output(data->gpio_pwen.gpio_num, 0);
		gpio_set_value(data->gpio_pwen.gpio_num, 0);
	}

	if (data->gpio_spken.gpio_num != -1) {
		gpio_direction_output(data->gpio_spken.gpio_num, 0);
		gpio_set_value(data->gpio_spken.gpio_num, 0);
	}

	if (data->sleep_flag != 0) {
		wake_unlock(&data->rp_wake_lock);//防止进入深度睡眼
	}
	return 0;
}



/*
static void  power_en_suspend(struct platform_device* pdev) {
	struct power_en_gpio* data = platform_get_drvdata(pdev);
	if (data->gpio_sdemmc.gpio_num != -1) {
		gpio_direction_output(data->gpio_sdemmc.gpio_num, 0);
		//gpio_set_value(data->gpio_sdemmc.gpio_num,0);
		gpio_free(data->gpio_sdemmc.gpio_num);
	}

	if (data->gpio_fan.gpio_num != -1) {
		gpio_direction_output(data->gpio_fan.gpio_num, 0);
		//gpio_set_value(data->gpio_sdemmc.gpio_num,0);
		gpio_free(data->gpio_fan.gpio_num);
	}
	if (data->gpio_pwen.gpio_num != -1) {
		gpio_direction_output(data->gpio_pwen.gpio_num, 0);
		//gpio_set_value(data->gpio_sdemmc.gpio_num,0);
		gpio_free(data->gpio_pwen.gpio_num);
	}
	if (data->gpio_spken.gpio_num != -1) {
		gpio_direction_output(data->gpio_spken.gpio_num, 0);
		//gpio_set_value(data->gpio_sdemmc.gpio_num,0);
		gpio_free(data->gpio_spken.gpio_num);
	}

//	return 0;
}

*/


static  int power_en_resume(struct platform_device* pdev) {
	//获取平台数据
	struct power_en_gpio* data = platform_get_drvdata(pdev);
	if (data->gpio_sdemmc.gpio_num != -1) {
		gpio_direction_output(data->gpio_sdemmc.gpio_num, 0);
		gpio_set_value(data->gpio_sdemmc.gpio_num, 0);
	}


	if (data->gpio_fan.gpio_num != -1) {
		gpio_direction_output(data->gpio_fan.gpio_num, 0);
		gpio_set_value(data->gpio_fan.gpio_num, 0);
	}

	if (data->gpio_pwen.gpio_num != -1) {
		gpio_direction_output(data->gpio_pwen.gpio_num, 0);
		gpio_set_value(data->gpio_pwen.gpio_num, 0);
	}

	if (data->gpio_spken.gpio_num != -1) {
		gpio_direction_output(data->gpio_spken.gpio_num, 0);
		gpio_set_value(data->gpio_spken.gpio_num, 0);
	}



	return 0;

}
static const struct of_device_id power_en_of_match[] = {

		{.compatible = "gpio_add_mark" },
		{ }
};



static struct platform_driver power_en_driver = {
	.probe = power_en_probe,
	.remove = power_en_remove,
	.driver = {
		.name = "gpio_driver",
		.of_match_table = of_match_ptr(power_en_of_match),
		.owner = THIS_MODULE,
	},
//	.suspend = power_en_suspend,
	.resume = power_en_resume,



};




static int __init mark_gpio_init(void)
{
	platform_driver_register(&power_en_driver);//注册函数
	printk("module init error *************************\n");
	return 0;
}
subsys_initcall(mark_gpio_init);
static void __exit  mark_gpio_exit(void)
{
	platform_driver_unregister(&power_en_driver);//取消注册
}
module_exit(mark_gpio_exit);
MODULE_LICENSE("GPL");

