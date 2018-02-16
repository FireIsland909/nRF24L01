# nRF24L01
/*
 * Transmiter.c
 *
 * Created: 2018-02-09 13:03:58
 * Author : Piotr
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "nRF24L01.h"

#define BIT(x) (1<<(x))
#define SETBITS(x,y) ((x)|=(y))
#define CLEARBITS(x,y) ((x)&=(~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))

#define W 1
#define R 0

uint8_t *data;

/* nRF24L01+ pinout:
CE - PC7
CSN - PC6
SCK - SCK (PB7)
MOSI - MOSI (PB5)
MISO - MISO (PB6)
IRQ - INT0 (PD2)
*/

void SPI_init(void)
{
	DDRC|= _BV(PC6)|_BV(PC7); //Pin CSN i CE jako wyjscie
	DDRB|=_BV(PB4) |_BV(PB5) |_BV(PB7); // Pin SS, MOSI pin SCK jako wyjście
	SPCR|=_BV(SPE) |_BV(MSTR) |_BV(SPR0) |_BV(SPIE); // SPI Enable, Tryb Master, CLK/16, Przerwanie
	//SPSR|=_BV(SPIF);// Flaga przerwania - transmisja zakonczona
	SPDR;//Skasuj flagę SPIF
	
	SETBIT(PORTC, 6); //CSN High
	CLEARBIT(PORTC, 7); //CE Low
	
}

char Write_Byte_SPI(unsigned char cData)
{
	SPDR = cData; // Wpisz dane do rejestru
	while(!(SPSR & (1<<SPIF)));//Poczekaj na koniec transmisji
	return SPDR; //Zwróć to co odebrano. (Kiedy wysyła, to jednocześnie odbiera). 
}

uint8_t GetReg(uint8_t reg)
{
	_delay_us(10);//Odczekanie, aby ostatnia komenda się zakonczyła.
	CLEARBIT(PORTC, 6); //Pin CSN w stan niski - Moudł nRF zaczyna nasłuchiwać
	_delay_us(10);
	Write_Byte_SPI(R_REGISTER + reg); //Ustaw nRF w tryb odczytywania, reg - ten rejestr zostanie odczytany
	_delay_us(10);
	reg = Write_Byte_SPI(NOP); // Wyślij sztuczny bajt (255) żeby w zamian dostać to co chcesz otrzymać wyżej. 
	_delay_us(10);
	SETBIT(PORTC, 6); //CSN w stan wysoki - koniec. 
	return reg; //Zwróc odczytany rejestr. 
}

uint8_t *Write_To_nRF(uint8_t readWrite, uint8_t reg, uint8_t *val, uint8_t antVal)
{
	//readWrtie - W lub R, reg - rejestr, *val - tablica danych, antVal - liczba INTów w pakiecie. 
	cli();
	
	if(readWrite == W) //W = 1, R = 0 (define na początku). 
	{
		reg = W_REGISTER + reg; //Dodaj nowy bit do rejestru.
	}
	
	static uint8_t ret[32]; //Tablica, która zostanie zwrócona na końcu.
	
	_delay_us(10);
	CLEARBIT(PORTC, 6);//CSN w stan niski
	_delay_us(10);
	Write_Byte_SPI(reg); //Ustaw nRF w stan odczytu albo zapisu. Zależnie przy było W czy R. 
	_delay_us(10);
	
	int i;
	for(i = 0; i < antVal; i++)
	{
		if(readWrite == R && reg != W_TX_PAYLOAD)
		{
			ret[i] = Write_Byte_SPI(NOP);
			_delay_us(10);
		}
		else
		{
			Write_Byte_SPI(val[i]);
			_delay_us(10);
		}
	}
	SETBIT(PORTC, 6); //CSN w stan wysoki
	sei();
	
	return ret; //Zwróc tablice.
}

void nRF24L01_init()
{
	_delay_ms(100);
	
	uint8_t val[5]; //Dane do przekazania funkcji Write_To_Nrf
	val[0] = 0x01;
	Write_To_nRF(W, EN_AA, val, 1);//Tryb zapisu, zapis do rejestru EN_AA, val - dane do zapisania, 1 - liczba bajtow
	
	val[0] = 0x01;
	Write_To_nRF(W, EN_RXADDR, val, 1);//Aktywny kanał 0 transmisji
	
	//Ustawienie rozmiaru adresu modul nRF (jak duzo zajmuje bajtow)
	val[0] = 0x03; 
	Write_To_nRF(W, SETUP_AW, val, 1);
	
	//Ustawienie czestotliwości
	val[0] = 0x01;//2,401GHz
	Write_To_nRF(W, RF_CH, val, 1);
	
	//Ustawienie trybu poboru mocy i szybkości
	val[0] = 0x07;
	Write_To_nRF(W, RF_SETUP, val, 1);
	
	
	//Adres odbiornika (5 bajtów)
	int i;
	for(i = 0; i <5; i++)
	{
		val[i] = 0x12;
	}
	Write_To_nRF(W, RX_ADDR_P0, val, 5); //Wcześniej był wybrany kanał 0, wiec ten adres został nadany dla kanalu 0. Mozna dodać inne adresy, żeby komunikować się z większą liczką modułów. nRF.
	
	//Adres nadajnika (5 bajtów)
	for(i = 0; i<5; i++)
	{
		val[i] = 0x12;
	}
	Write_To_nRF(W, TX_ADDR, val, 5);
	
	//Ustawienie rozmiaru danych
	val[0] = 5; //5 bajtów na pakiet
	Write_To_nRF(W, RX_PW_P0, val, 1);
	
	//Ustawienie rejestru CONFIG. 
	val[0] = 0x1E; //bit O = 0 -> Transmiter, bit 0 = 1 -> Receiver, bit 1 = 1 -> Power Up, bit 4 = 1 ->mask_MAX_RT. IRQ nie jest wyzwalane, jeśli transmisja się nie powwiedzie. 
	Write_To_nRF(W, CONFIG, val, 1);
	
	//Liczba prób i opóźnienia pomiędzy próbami
	val[0] = 0x2F; // bit 2 -> Opoznienie 750us, F -> liczba prób (15). Opoznienie musi wynosi minimum 500us przy 250kbps oraz jesli dane maja wiecej niż 5 bajtów przy predkosci 1Mbps i jeśli mają powyżej 15 bajtów przy 2Mbps. 
	Write_To_nRF(W, SETUP_RETR, val, 1);
	
	//Opoznienie konieczne zeby przejsc w tryb StandBy (niski stan na CE)
	_delay_ms(100);
	
}

void Transmit_payload(uint8_t *W_buff)
{
	Write_To_nRF(R, FLUSH_TX, W_buff, 0);//Wyczyszczenie rejestru ze starych danych
	Write_To_nRF(R, W_TX_PAYLOAD, W_buff, 5);
	sei();
	
	_delay_ms(10);//Konieczne opoznienie zeby pracowac po zaladowaniu do nRF danych
	SETBIT(PORTC, 7);//CE w stan wysoki - rozpocznij transmisje
	_delay_us(20);
	CLEARBIT(PORTC, 7); //CE w stan niski - zakoncz transmisje. 
	_delay_ms(10);
	
}

void Receive_payload(void)//Funkcja odbiera dane
{
	sei();
	SETBIT(PORTC, 7);//CE w stan wysoki
	_delay_ms(1000);
	CLEARBIT(PORTC, 7);//CE w stan niski
	//cli();
}

void reset(void)//Po kazdym odebraniu/wyslaniu danych IRQ musi zostać skasowane w nRF. Robi to poniższa funkcja. 
{
	_delay_us(10);
	CLEARBIT(PORTC, 6);//CSN w stan niski
	_delay_us(10);
	Write_Byte_SPI((uint8_t)(W_REGISTER + STATUS));//Zapisz do rejestru STATUS.
	_delay_us(10);
	Write_Byte_SPI((uint8_t)(0x70));//Skasuj wszystkie IRQ w rejestrze STATUS. 
	_delay_us(10);
	SETBIT(PORTC, 6);//CSN w stan wysoki. 
}

void INT0_interrupt_init(void)//Przerwanie generowane po pomyslnej transmisji lub odbiorze pakietu. Pin IRQ zmienia swoj stan, co powoduje przerwanie na pinie INT0 procesora. 
{
	DDRD|=_BV(PD2);//INT0 jak wejscie
	PORTD|=_BV(PD2);//Podciaganie wejscia INT0
	EIMSK|=_BV(INT0);//Zezwolenie na przerwanie INT0; 
	EICRA|=_BV(ISC01) |_BV(ISC00);//Zbocze opadające
}

int main(void)
{
	DDRA|=_BV(PA3) |_BV(PA2) |_BV(PA1) |_BV(PA0);// LED
	PORTB|=_BV(PB4);
	uint8_t W_buffer[6];
	uint8_t a=0;

	SPI_init();
	nRF24L01_init();
	INT0_interrupt_init();
  
	sei();
	
    while (1) 
    {
	
      for(uint8_t i = 0; i<5; i++)
      {
        W_buffer[i] = 0x93; //5 razy wyslij wartosc 0x93
      }

      Transmit_payload(W_buffer);

      PORTA&=~_BV(PA1); //Zaświec LED blad
      _delay_ms(50);
      PORTA|=_BV(PA1);//Wylacz LED blad
      _delay_ms(50);

      reset();
    }
}


ISR(INT0_vect)//Obluga przerwania INT0 (Pomyslnie odebrane lub wyslane dane)
{
	CLEARBIT(PORTA, 0); // Zapal LED
	_delay_ms(100);
	SETBIT(PORTA, 0); //Zgas LED
	
}

