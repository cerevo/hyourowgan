#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* MCU support. */
#include "PMU_TZ10xx.h"
#include "TMR_TZ10xx.h"

#include "TZ01_system.h"

#include "pwm_out.h"

extern uint32_t SystemCoreClock;
extern TZ10XX_DRIVER_TMR Driver_ADVTMR0;
extern TZ10XX_DRIVER_TMR Driver_ADVTMR1;

const static TZ10XX_DRIVER_TMR *advtmr[] = { &Driver_ADVTMR0, &Driver_ADVTMR1, NULL };

static uint32_t INPUT_CLOCK = 0;


bool pwm_out_init(void)
{
    //タイマーの入力クロックを取得
    int div = Driver_PMU.GetPrescaler(PMU_CD_PPIER0);
    //// システムコアクロック / クロックドメイン`PMU_CD_PPIER0'での分周
    INPUT_CLOCK = (SystemCoreClock / div) / TIMER_CLK_DIV;
    
    for (int i = 0; advtmr[i] != NULL; i++) {
        /* AdvTMR0 */
        advtmr[i]->Initialize(NULL, 0);
        advtmr[i]->PowerControl(ARM_POWER_FULL);
        advtmr[i]->Configure(16, TMR_COUNT_MODE_PERIODIC, TIMER_CLK_DIV);
        //PWMの設定
        advtmr[i]->ConfigureTFF(TMR_TFF_MODE_CMP_TOGGLE, false, false);    //コンペアマッチしたら出力をトグルするよ
        advtmr[i]->EnableCompare(true);
    }
    
    return true;
}

bool pwm_out_start(PWM_OUT_CH ch, uint32_t clock, float duty)
{
    uint16_t cnt, cmp;
    
    if (INPUT_CLOCK == 0) {
        return false;   //初期化終わってない
    }
    
    if ((clock < PWM_MIN_FREQ) || (clock > PWM_MAX_FREQ)) {
        return false;   //クロックが範囲外
    }
    
    if ((duty < 0.0) || (duty > 1.0)) {
        return false;   //デューティが範囲外
    }
    
    if (duty == 1.0) {
        //Duty 100%時は周波数を下げてたうえでHiに張り付ける
        advtmr[ch]->Stop();
        advtmr[ch]->ConfigureTFF(TMR_TFF_MODE_CMP_ONE, false, false);
        advtmr[ch]->SetCompareValue(0, false);
        advtmr[ch]->Start(65535);
    } else {
        cnt = INPUT_CLOCK / clock;
        cmp = cnt * duty;
        advtmr[ch]->Stop();
        advtmr[ch]->ConfigureTFF(TMR_TFF_MODE_CMP_TERM_TOGGLE, false, false);
        advtmr[ch]->SetCompareValue(cmp, false);
        advtmr[ch]->Start(cnt);
    }
    advtmr[ch]->EnableTFF(true);
    return true;
}

bool pwm_out_stop(PWM_OUT_CH ch)
{
    advtmr[ch]->Stop();
    advtmr[ch]->EnableTFF(false);
    return true;
}

