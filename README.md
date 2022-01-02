# Ultrasonic_following_car #
上海交通大学IEEE及ai专业电子电路实验课程的大作业选题之一。由于是新加入的项目，难度较大，且与课程教授知识相去甚远，没有电子制作经验的同学可能比较难以完成，特此留下基于stm32的控制代码以供学弟学妹参考。
## 0. 写在前面 ##
此项目由李森、陆晟宇、汪昊阳（拼音顺序）小组制作完成，所有版权归此三人所有。作为开源项目，欢迎各位对代码进行指正或修改，也欢迎大家和作者联系交流。当然，更希望这份代码，或者说这套解决方案，可以帮助到学弟学妹们。但是，我们并不鼓励学弟学妹们直接照抄这套方案，应付老师，更加不希望这套方案限制了大家的思路。提供这套方案只是为了让大家有一个基本思路，不至于一头雾水地做项目。做项目是一件非常能够带来成就感和满足感的事情，希望大家能广阔地发挥思路，享受上大学以来第一个比较正式的项目。
## 1. 硬件基本说明 ##
### (a) 传感器电路部分 ###
    to be updated
### (b) 控制器部分 ###
基于stm32f103c8t6最小系统板开发。

## 2. 系统运行框架 ##
小车部分运行框架主要有两个部分组成：`main`函数主循环控制运动以及`timer1`的400kHz中断用于电压采样读取接受器探头的峰峰值。为保证传感器反馈保持在线的状态，我们使用`timer1`的定时器中断作为最高优先级中断。同时读取电压值方式采用`ADC`的中断进行读取。由于``HAL``库的一些编程特性，调用`ADC`中断回调函数后会自动关闭``ADC``的中断，因此我们将两者的比较次数都改为`239.5`个周期，防止两者互相抢中断的情况发生。另外，实测中发现由于电压采样所需频率过高，如果用比较低频的定时器中断触发小车运动控制函数，非常容易出现无法进入中断的情况，故只能将其放置于主函数的```while```循环中,不断地调用以确保能进入主控制函数。

## 3. 基本模块实现 ##

### (a) 距离感知 ###

小车主要通过判断两个接受端探头电压的峰峰值大小以实现距离的感知。通过示波器观察接收端信号，其信号为非常规整的40kHz正弦波，其峰峰值随距离变化，大致范围在`0.1v-3.2v`。我们认为这足以进行达到跟随的要求，便舍弃了接收端电路的设计。同时，通过`stm32`自带的数模转换器进行对电压值的读取，并通过公式
```C
voltage = (float)HAL_ADC_GetValue(&hadc1) / 4096.0f * VCC
```
将`ADC`的返回值转化为电压值。

需要注意的是，`ADC`读取的电压值为实时的真实电压，并不是信号的峰峰值。因此，我们采用在一个周期内取10个点，即以400kHz为频率进行采样，计算电压最大差值作为距离传感器的反馈量。实际应用中，此方法效果较好，且不过于占用单片机资源。

### (b) 电机变速控制

由于课程所提供的直流电机驱动模块的特性，我们通过`PWM`（Pulse Width Modulation）实现电机的速度控制。通过`timer3`产生`PWM`波改变输出方波占空比（duty）控制电机转动的速度。距离实现代码如下。
```C
void SetChassisSpeed(int16_t left, int16_t right) {
		//  left motor
		if (left > 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, left);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
		}
		else if (left == 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
		}
		else if (left < 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, -left);
		}
		
		// right motor
		if (right > 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, right);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
		}
		else if (right == 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
		}
		else if (right < 0) {
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
			  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, -right);
		}
}
```
## 4. 运行逻辑与控制算法
### (a) 工况分类
    to be updated
### (b) 转向PID控制
    to be updated
### (c) 控制逻辑缺陷与算法的问题
    to be update
## 5. 后续改进空间

