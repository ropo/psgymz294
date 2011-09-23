#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>

////////////////////////////////////////////////////////////////////////////
// 定数定義
////////////////////////////////////////////////////////////////////////////
#define FOSC F_CPU
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)

// PORTD(7bit)
#define YM_WRCS		0b0001000
#define YM_A		0b0010000

// PORTA(3bit)
#define YM_IC		0b001

// トラック数
#define MAX_CH		3

#define MAKEWORD(h,l)	(((h)<<8)|(l))

// COMMAND
enum {
	// 周波数の設定
	 YM_TP_A_LOW		// $00:
	,YM_TP_A_HIGH		// $01:
	,YM_TP_B_LOW		// $02:
	,YM_TP_B_HIGH		// $03:
	,YM_TP_C_LOW		// $04:
	,YM_TP_C_HIGH		// $05:

	// ノイズ周波数の設定
	,YM_NOISE			// $06:

	// ミキサー
	,YM_MIXIER			// $07:

	// ボリューム
	,YM_VOL_A			// $08:
	,YM_VOL_B			// $09:
	,YM_VOL_C			// $0A:

	// エンベロープ周波数設定
	,YM_ENB_LOW			// $0B:
	,YM_ENB_HIGH		// $0C:

	// エンベロープ
	,YM_ENBELOPE		// $0D:

	//////////////
	// 拡張命令
	//////////////
	// ADSR 設定
	,YMEX_ADSR_A = 0x10	// 連続で次のコマンドを送る
						// アタックカウント上位バイト
						// アタックカウント下位バイト
						// ディケイカウント上位バイト
						// ディケイカウント下位バイト
						// サスティンボリューム 0-15
						// リリースカウント上位バイト
						// リリースカウント下位バイト
	,YMEX_ADSR_B
	,YMEX_ADSR_C

	// ビブラート設定
	,YMEX_VIBRATO_A		// 連続で次のコマンドを送る
						// ステップカウント上位バイト
						// ステップカウント下位バイト
						// ディレイカウント上位バイト
						// ディレイカウント下位バイト
						// 深さ
	,YMEX_VIBRATO_B
	,YMEX_VIBRATO_C

	// 発音一括設定
	,YMEX_TP_A			// 連続で次のコマンドを送る
						// TP_LOW (0-255)
						// TP_HIGH(0-15)  0x10:ビブラートとADSRをリセットする(再発音)
	,YMEX_TP_B
	,YMEX_TP_C

	// パート設定
	,YMEX_PERCUSSION_A	// パーカッション指定  eYMEX_FLAGS.YMEX_PERCUSSION_FLAG_*
	,YMEX_PERCUSSION_B	// パーカッショントラックの場合はノートオフが無効化される
	,YMEX_PERCUSSION_C

	// 拡張ノイズ周波数設定
	,YMEX_NOISE_A		// YM_NOISE | eYMEX_FLAGS.YMEX_NOISE_FLAG_ADSR_*
	,YMEX_NOISE_B		//
	,YMEX_NOISE_C		//


	// 0x80～ 7ビットをそのままYMZへダイレクト通知
}eYM_COMMAND;

enum {
	 YMEX_PERCUSSION_FLAG_OFF		= 0x00	// パーカッションチャンネル無効化
	,YMEX_PERCUSSION_FLAG_ON		= 0x01	// パーカッションチャンネル登録
	,YMEX_PERCUSSION_FLAG_USEADSR	= 0x81	// ADSR 有効化
	,YMEX_NOISE_FLAG_ADSR_RESET		= 0x80	// ADSR 進行をリセット
	,YMEX_NOISE_FLAG_ADSR_RELEASE	= 0xc0	// データにASDRのリリースタイムを追加する(上位バイト,下位バイト)
}eYMEX_FLAGS;

#define false	0
#define true	1
typedef char			bool;
typedef	unsigned char	u8;
typedef	  signed char	s8;
typedef	unsigned short	u16;
typedef	  signed short	s16;

typedef u8				updateflag_t;
////////////////////////////////////////////////////////////////////////////
// 列挙定義
////////////////////////////////////////////////////////////////////////////
enum tagCH_UPDATE_FLAG
{
	 CH_UPDATE_FLAG_NONE		= 0			// イベントなし
	,CH_UPDATE_FLAG_VOLUME		= 1 << 0	// ボリューム変更
	,CH_UPDATE_FLAG_FREQUENCY	= 1 << 1	// 周波数変更
}eCH_UPDATE_FLAG;

enum tagADSR_MODE
{
	 ADSR_MODE_DISABLE = 0
	,ADSR_MODE_ATTACK
	,ADSR_MODE_DECAY
	,ADSR_MODE_SUSTAIN
	,ADSR_MODE_RELEASE
	,ADSR_MODE_ENABLED
}eADSR_MODE;

////////////////////////////////////////////////////////////////////////////
// 構造体定義
////////////////////////////////////////////////////////////////////////////
// ADSR
typedef struct tagADSR
{
	char	mode;		// eADSR_MODE
	int		counter;	// デクリメントカウンタ
	int		attack;		// アタック減衰カウンタ
	int		decay;		// ディケイ減衰カウンタ
	u8		sustain;	// サスティンボリューム 0-15
	int		release;	// リリース減衰カウンタ
}ADSR;

// ビブラート
typedef struct tagVIBRATO
{
	bool 	enabled;	// 有効なら true
	int		counter;	// デクリメントカウンタ
	int		rate;		// 速さ
	int		delay;		// ディレイ
	u8		depth;		// 深さ
	u16		base_freq;	// 基本tp
	u8		step;		// 深さまでデクリメントカウンタ
	char	vector;		// 加算値
}VIBRATO;

// チャンネル情報
typedef struct tagCHINFO
{
	// 基本情報
	updateflag_t updateFlag;	// eCH_UPDATE_FLAG
	u8		volume;				// 現在の発音ボリューム 0-15
	int		tp;					// 周波数
	u8		velocity;			// ノートベロシティ     0-15
	bool	isHWEnbelope;		// ハードウェアエンベロープが有効か
	bool	isPercussion;		// パーッカッションチャンネルか？ (trueだとノートオフを行わない)

	// エフェクト
	ADSR	adsr;				// ADSR
	VIBRATO	vibrato;			// ビブラート
}CHINFO;

////////////////////////////////////////////////////////////////////////////
// グローバル変数
////////////////////////////////////////////////////////////////////////////
CHINFO gCH[MAX_CH];


////////////////////////////////////////////////////////////////////////////
// プログラム
////////////////////////////////////////////////////////////////////////////
// データ書き込み
inline void putData( u8 addr, u8 data )
{
	// アドレス指定
	PORTD  = 0;
	PORTB  = addr;
	PORTD  = YM_WRCS;

	// データ出力
	PORTD  = YM_A;
	PORTB  = data;
	PORTD |= YM_WRCS;
}

/* USART設定 */
inline void usart_init( void )
{
    UBRRH = (unsigned char)(MYUBRR>>8);
    UBRRL = (unsigned char)(MYUBRR);
    UCSRB = (1<<RXEN)|(1<<TXEN);	// 送受信可
    UCSRC = (0<<USBS)|(3<<UCSZ0);   // 1stop bit, 8 bit
}

// 1バイト送信
inline void uart_putch(const unsigned char data)
{
    loop_until_bit_is_set(UCSRA, UDRE);	// 送信バッファが空くまで待機
    UDR = data;							// 送信
}

// 文字列送信
inline void uart_puts(const char *str)
{
	while( *str )
		uart_putch( *str++ );
}

// 受信データがあれば !0 を返す
inline char isSioRecived( void ) {
	return bit_is_set( UCSRA, RXC );
}

// 1バイトの受信(ブロッキング)
inline unsigned char sio_getch( void ) {
    loop_until_bit_is_set( UCSRA, RXC );
    return UDR;
}

// 1バイトの受信(ノンブロッキング)
inline bool sio_getch_nb( unsigned char *pChar ) {
	if( isSioRecived() ) {
		*pChar = UDR;
		return 1;
	}
	return 0;
}

// 16進数表示
inline void sio_putHex( u8 c )
{
	u8 d;
	d = c >> 4;
	if( d > 10 )	d = 'a' + d - 10;
	else			d = '0' + d;
	uart_putch( d );

	d = c & 0x0f;
	if( d > 10 )	d = 'a' + d - 10;
	else			d = '0' + d;
	uart_putch( d );
	uart_putch( ' ' );
}

inline void SetADSR( u8 ch, int a/*マイナスでADSL解除*/, int d, u8 s, int r )
{
	ADSR *pADSR = &gCH[ch].adsr;

	if( a >= 0 ) {
		pADSR->mode = ADSR_MODE_ENABLED;
		pADSR->attack	= a;
		pADSR->decay 	= d;
		pADSR->sustain	= s;
		pADSR->release	= r;
	}else{
		pADSR->mode = ADSR_MODE_DISABLE;
	}
}

inline void SetVibrato( u8 ch, int rate/*マイナスでADSL解除*/, int delay, u8 depth )
{
	VIBRATO *pVibrato = &gCH[ch].vibrato;

	if( rate >= 0 ) {
		pVibrato->enabled	= true;
		pVibrato->rate		= rate;
		pVibrato->delay		= delay;
		pVibrato->depth		= depth;
		pVibrato->counter	= delay;
		pVibrato->step		= depth / 2;	// はじめは半弧
		pVibrato->vector	= -1;
	}else{
		pVibrato->enabled = false;
	}

}

inline void SetTP( int ch, int tp )
{
	CHINFO *pCI = &gCH[ch];
	pCI->tp = tp & 0x0fff;
	pCI->updateFlag |= CH_UPDATE_FLAG_FREQUENCY;
}
inline int GetTP( int ch )
{
	return gCH[ch].tp;
}


// ADSR処理
inline void UpdateADSR( void )
{
	for( int ch=0; ch<MAX_CH; ch++ ) {
		CHINFO *pCI = &gCH[ch];
		ADSR *pADSR = &pCI->adsr;
		int counter = (pADSR->counter)?pADSR->counter-1:0;

		// ADSR 更新
		int vol = -1;
		if( counter == 0 ) {
			switch( pADSR->mode )
			{
				// アタック中
				case ADSR_MODE_ATTACK:
						vol = pCI->volume+1;
						if( vol >= (pCI->velocity&0x0f) ) {
							vol =(pCI->velocity&0x0f);
							pADSR->mode = ADSR_MODE_DECAY;
							counter = pADSR->decay;
						}else{
							counter = pADSR->attack;
						}break;
				// ディケイ中
				case ADSR_MODE_DECAY:
						vol = pCI->volume-1;
						if( vol <= pADSR->sustain ) {
							vol = pADSR->sustain;
							pADSR->mode = ADSR_MODE_SUSTAIN;
							counter = pADSR->release;
						}else{
							counter = pADSR->decay;
						}break;
				// リリース中
				case ADSR_MODE_RELEASE:
						vol = pCI->volume-1;
						if( vol <= 0 ) {
							vol = 0;
							pADSR->mode = ADSR_MODE_ENABLED;
							counter = pADSR->attack;
						}else{
							counter = pADSR->release;
						}break;
			}
		}
		pADSR->counter = counter;

		// 音量変更があった
		if( vol != -1 ) {
			vol &= 0x0f;
			if( pCI->volume != vol ) {
				pCI->updateFlag |= CH_UPDATE_FLAG_VOLUME;
				pCI->volume = vol;
			}
		}
	}
}

// ビブラート処理
inline void UpdateVibrato( void )
{
	for( int ch=0; ch<MAX_CH; ch++ ) {
		VIBRATO *pVibrato = &gCH[ch].vibrato;
		if( pVibrato->enabled == false )
			continue;

		int counter = (pVibrato->counter)?pVibrato->counter-1:0;
		if( counter == 0 ) {
			int tp = GetTP(ch);
			counter = pVibrato->rate;

			if( ++pVibrato->step >= pVibrato->depth ) {
				pVibrato->step = 0;
				pVibrato->vector = -pVibrato->vector;
			}
			tp += pVibrato->vector;
			if( tp < 0 )				tp = 0;
			else if( tp >= 0x1000 )		tp = 0xfff;
			SetTP( ch, tp );
		}
		pVibrato->counter = counter;
	}
}

inline void Update( void )
{
	UpdateADSR();
	UpdateVibrato();

	// 更新フラグ処理
	for( int ch=0; ch<MAX_CH; ch++ ) {
		CHINFO *pCI = &gCH[ch];
		updateflag_t flag = pCI->updateFlag;
		pCI->updateFlag = CH_UPDATE_FLAG_NONE;

		// ボリューム変更
		if( flag & CH_UPDATE_FLAG_VOLUME ) {
			u8 value = pCI->volume & 0x0f;
			if( pCI->isHWEnbelope )
				value |= 0x10;
			putData( YM_VOL_A + ch, value );
		}

		// 周波数変更
		if( flag & CH_UPDATE_FLAG_FREQUENCY ) {
			putData( YM_TP_A_LOW  + ch*2, pCI->tp&0xff );
			putData( YM_TP_A_HIGH + ch*2, pCI->tp>>8 );
		}
	}
}

void SetVolume( char ch, char vol/*0-31*/ )
{
	CHINFO *pCI = &gCH[(int)ch];

	// エンベ？
	pCI->isHWEnbelope = (vol&0x10)?true:false;
	vol &= 0x0f;
	pCI->velocity = vol;

	// ビブラートが有効か？
	if( pCI->vibrato.enabled == true ) {
		VIBRATO *pVibrato = &pCI->vibrato;
		pVibrato->counter	= pVibrato->delay;
		pVibrato->step		= pVibrato->depth / 2;
		pVibrato->vector	= -1;
	}

	// ADSL が有効か？
	int volume = -1;
	if( pCI->adsr.mode != ADSR_MODE_DISABLE ) {
		ADSR *pADSR = &pCI->adsr;

		// 発音か消音か？
		if( vol ) {
			/////////
			// 発音
			/////////
			if( pADSR->attack ) {
				// アタックへ
				pADSR->mode = ADSR_MODE_ATTACK;
				pADSR->counter = pADSR->attack;
				volume = 0;
			}else if( pADSR->decay ) {
				// ディケイへ
				pADSR->mode = ADSR_MODE_DECAY;
				pADSR->counter = pADSR->decay;
				volume = vol;
			}else{
				// サスティンへ
				pADSR->mode = ADSR_MODE_SUSTAIN;
				volume = pADSR->sustain;
			}
		}else{
			/////////
			// 消音
			/////////
			if( pADSR->release ) {
				// リリースモードへ
				pADSR->mode = ADSR_MODE_RELEASE;
				pADSR->counter = pADSR->release;
			}else{
				// リリースタイムがないならそのまま消音
				pADSR->mode = ADSR_MODE_ENABLED;
				if( pCI->isPercussion )
					return;	// パーカッションチャンネルなら止めない
				volume = 0;
			}
		}
	}else{
		// ADSL を使用していないので直書き込み
		if( vol==0 && pCI->isPercussion )
			return;	// パーカッションチャンネルなら止めない
		volume = vol;
	}

	// ボリュームの変更があった
	if( volume != -1 ) {
		pCI->volume = volume;
		pCI->updateFlag |= CH_UPDATE_FLAG_VOLUME;
	}
}
inline u8 GetVelocity( u8 ch )
{
	return gCH[ch].velocity;
}

// メイン処理
int main(void)
{
	memset( gCH, 0, sizeof(gCH) );
	for( int i=0; i<MAX_CH; i++ ) {
		gCH[i].volume = -1;
	}

	// ポートA 設定
	PORTA = 0b000;
	DDRA  = YM_IC;

	// ポートB 設定
    PORTB = 0b00000000;			// 全て出力をOFF
    DDRB  = 0b11111111;			// 全て出力設定

	// ポートD 設定
	PORTD = 0b0000000;			// 全て出力をOFF
	DDRD  = YM_WRCS | YM_A;		// 出力設定

	// UART初期化
	usart_init();

	// 初期化
	PORTA = YM_IC;	// リセット解除
	putData( YM_MIXIER, 0b00111000 );


	// メインループ
	u8 rx[8];
	char readed = 0;
	for(;;) {
		if( sio_getch_nb( &rx[(int)readed] ) ) {
			readed++;
			if( readed == 2 ) {
				readed = 0;

				// コマンド別処理
				switch( rx[0] )
				{
					case YM_TP_A_LOW:
					case YM_TP_A_HIGH:
					case YM_TP_B_LOW:
					case YM_TP_B_HIGH:
					case YM_TP_C_LOW:
					case YM_TP_C_HIGH:{
							int *pTP = &gCH[rx[0]/2].tp;
							if( rx[0] & 1 )		*pTP = (*pTP & 0x0ff) + (rx[1]<<8);
							else				*pTP = (*pTP & 0xf00) + (rx[1]<<0);
							putData( rx[0], rx[1] );
							}break;
					case YM_VOL_A:
					case YM_VOL_B:
					case YM_VOL_C:
							SetVolume( rx[0]-YM_VOL_A, rx[1] );
							break;
					case YMEX_ADSR_A:
					case YMEX_ADSR_B:
					case YMEX_ADSR_C:
							rx[2] = sio_getch();
							rx[3] = sio_getch();
							rx[4] = sio_getch();
							rx[5] = sio_getch();
							rx[6] = sio_getch();
							rx[7] = sio_getch();
							SetADSR( rx[0]-YMEX_ADSR_A, MAKEWORD(rx[1],rx[2]), MAKEWORD(rx[3],rx[4]), rx[5], MAKEWORD(rx[6],rx[7]) );
							break;
					case YMEX_VIBRATO_A:
					case YMEX_VIBRATO_B:
					case YMEX_VIBRATO_C:
							rx[2] = sio_getch();
							rx[3] = sio_getch();
							rx[4] = sio_getch();
							rx[5] = sio_getch();
							SetVibrato( rx[0]-YMEX_VIBRATO_A, MAKEWORD(rx[1],rx[2]), MAKEWORD(rx[3],rx[4]), rx[5] );
							break;
					case YMEX_TP_A:
					case YMEX_TP_B:
					case YMEX_TP_C:{
							rx[2] = sio_getch();
							u8 ch = rx[0]-YMEX_TP_A;
							SetTP( ch, MAKEWORD(rx[2]&0x0f, rx[1]) );
							if( rx[2] & 0x10 )
								SetVolume( ch, GetVelocity( ch ) );
							}break;
					case YMEX_PERCUSSION_A:
					case YMEX_PERCUSSION_B:
					case YMEX_PERCUSSION_C:{
							u8 ch = rx[0]-YMEX_PERCUSSION_A;
							if( rx[1] & YMEX_PERCUSSION_FLAG_USEADSR )
								SetADSR( ch, 0, 0, GetVelocity(ch), 1 );
							gCH[ch].isPercussion = rx[1] & YMEX_PERCUSSION_FLAG_ON;
							}break;
					case YMEX_NOISE_A:
					case YMEX_NOISE_B:
					case YMEX_NOISE_C:{
							u8 ch = rx[0]-YMEX_NOISE_A;
							putData( YM_NOISE, rx[1]&0x1f );
							if( rx[1] & YMEX_NOISE_FLAG_ADSR_RELEASE ) {
								rx[2] = sio_getch();
								rx[3] = sio_getch();
								gCH[ch].adsr.release = MAKEWORD(rx[2],rx[3]);
							}
							if( rx[1] & YMEX_NOISE_FLAG_ADSR_RESET )
								SetVolume( ch, GetVelocity( ch ) );
							}break;
					default:putData( rx[0]&0x7f, rx[1] );
							break;
				}
			}
		}

		// 更新
		Update();
	}
}

