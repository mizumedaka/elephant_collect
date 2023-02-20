#include <CRSLib/Can/RM0008/include/can_manager.hpp>
#include <CRSLib/Can/RM0008/include/filter_manager.hpp>
#include <CRSLib/Can/RM0008/include/letterbox.hpp>
#include <CRSLib/Can/RM0008/include/pillarbox.hpp>
#include <CRSLib/Can/RM0008/include/handle.hpp>

#include "main.h"

using namespace CRSLib::IntegerTypes;
using namespace CRSLib::Can;
using namespace CRSLib::Can::RM0008;


extern "C"
{
	extern TIM_HandleTypeDef htim1;

	void main_cpp(CAN_HandleTypeDef *const hcan)
		{
			*hcan = Implement::crslib_default_can(CAN1);
			HAL_CAN_DeInit(hcan);
			CanManager can_manager{hcan};

			FilterManager::dynamic_initialize();

			Filter<FilterWidth::bit32, FilterMode::mask> filter1 =
				{
					.masked32 =
					{
					.id = {0x500, 0x0, false, false},
					.mask = {max_std_id, max_ext_id, true, true}
					}
				};

			Filter<FilterWidth::bit32, FilterMode::mask> filter2 =
				{
					.masked32 =
						{
							.id = {0x501, 0x0, false, false},
							.mask = {max_std_id, max_ext_id, true, true}
						}
				};

			FilterManager::ConfigFilterArg<FilterWidth::bit32, FilterMode::mask> filter_arg1
				{
					.filter = filter1,
					.fifo = FifoIndex::fifo0,
					.filter_match_index = 0, // なんでもいい。
					.activate = true
				};

			FilterManager::ConfigFilterArg<FilterWidth::bit32, FilterMode::mask> filter_arg2
				{
					.filter = filter2,
					.fifo = FifoIndex::fifo1,
					.filter_match_index = 0, // なんでもいい。
					.activate = true
				};

			FilterManager::config_filter_bank(15, filter_arg1, filter_arg2);

			HAL_CAN_Start(hcan);

			//CH1,2,3,4 は PA8,9,10,11 に 対応

			//PWM生成
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

			volatile int dummy = 0;
			while(true)
			{
				dummy = 1;

				constexpr auto open_or_close = [](const bool open, const auto ch1, const auto ch2)
				{
					constexpr unsigned int open_duty = 1100;
					constexpr unsigned int close_duty = 300;

					if(open) //オープン
						{
							__HAL_TIM_SET_COMPARE(&htim1, ch1, open_duty);
							__HAL_TIM_SET_COMPARE(&htim1, ch2, open_duty);
						}
					else //クローズ
						{
							__HAL_TIM_SET_COMPARE(&htim1, ch1, close_duty);
							__HAL_TIM_SET_COMPARE(&htim1, ch2, close_duty);
						}
				};

				bool open;

				if(!can_manager.letterbox0.empty())
					{
						RxFrame rx_frame{};
						can_manager.letterbox0.receive(rx_frame);
						open = rx_frame.data[0];
					}

				// 上段
				open_or_close(open, TIM_CHANNEL_1, TIM_CHANNEL_2);
				// 下段
				open_or_close(open, TIM_CHANNEL_3, TIM_CHANNEL_4);
			}
		}
}
