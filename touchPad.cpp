#include "daisy_seed.h"
#include "daisysp.h"
#include "per/spi.h"

#include "Envelope.h"
#include "lerp.h"

#define buffSize 6
#define packetStart 255

#define numNotes 2

#define noteThresh 15

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

SpiHandle dmaSpi;
SpiHandle::Config spi_config;

Oscillator myOsc,myOsc2;

// buffer for receiving data

uint8_t DMA_BUFFER_MEM_SECTION rx_buffer[buffSize];

//buffer macros

#define NOTE_ID 0UL
#define NOTE_X 1UL
#define NOTE_Y 2UL
#define NOTE_Z 3UL

#define STATE_OFF 0
#define STATE_ATT 1
#define STATE_HOLD 2
#define STATE_REL 3

typedef struct
{
	uint8_t x,y,z;
	uint8_t state;
}notes;

notes myNotes[numNotes];

#define lerpAmt 0.01
lerpF zLerp,xLerp;
lerpF zLerp1,xLerp1;



static void callbackFunction(void* context, SpiHandle::Result result)
{
	SpiHandle* transport = static_cast<SpiHandle*>(context);
	
	if(result == SpiHandle::Result::OK)
	{
		for(uint8_t i = 0;i < numNotes;i++)
		{
			uint8_t index = i * 3;
			myNotes[i].x = rx_buffer[index];
			myNotes[i].y = rx_buffer[index + 1];
			myNotes[i].z = rx_buffer[index + 2];
		}
		// notes* currentNote = &myNotes[rx_buffer[NOTE_ID]];

		// currentNote->state = rx_buffer[NOTE_ID];
		// currentNote->x = rx_buffer[NOTE_X];
		// currentNote->y = rx_buffer[NOTE_Y];
		// currentNote->z = rx_buffer[NOTE_Z];

		transport->DmaReceive(rx_buffer,buffSize,nullptr,callbackFunction,context);
	}
	// else
	// {
	// 	transport->DmaReceive(rx_buffer,5,nullptr,callbackFunction,context);
	// }

}



void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{

	float amp1,amp2;
	amp1 = zLerp.process((myNotes[0].z / 60.f)*(myNotes[0].z / 60.f));
	amp2 = zLerp1.process((myNotes[1].z / 60.f)*(myNotes[1].z / 60.f));

	myOsc.SetAmp(amp1);
	myOsc.SetFreq(xLerp.process(50.0f + myNotes[0].x) * 2.0f);

	myOsc2.SetAmp(amp2);
	myOsc2.SetFreq(xLerp1.process(50.0f + myNotes[1].x) * 2.0f);

	// myOsc.SetAmp(myNotes[0].z / 75.f);
	// myOsc.SetFreq(myNotes[0].x);


	for(size_t i = 0; i < size; i++)
	{
		float sum = 0;
		sum +=myOsc.Process();
		sum +=myOsc2.Process();
		out[0][i] = out[1][i] = 0.55 * sum;
		// out[0][i] = in[0][i];
		// out[1][i] = in[1][i];
	}
}

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.SetLed(false);

	zLerp.setAmt(lerpAmt);
	xLerp.setAmt(lerpAmt);

	zLerp1.setAmt(lerpAmt);
	xLerp1.setAmt(lerpAmt);


	float sampleRate = hw.AudioSampleRate();



	myOsc.Init(sampleRate);
	myOsc.SetAmp(1.0);
	myOsc.SetFreq(200);
	myOsc.SetWaveform(Oscillator::WAVE_SIN);

	myOsc2.Init(sampleRate);
	myOsc2.SetAmp(1.0);
	myOsc2.SetFreq(200);
	myOsc2.SetWaveform(Oscillator::WAVE_SIN);


	for(int i = 0;i<numNotes;i++)
	{
		myNotes[i].x=0;
		myNotes[i].y=0;
		myNotes[i].z=0;
	}



	//spi setup
	spi_config.periph = SpiHandle::Config::Peripheral::SPI_1;
    spi_config.mode   = SpiHandle::Config::Mode::SLAVE;
    spi_config.direction  = SpiHandle::Config::Direction::TWO_LINES_RX_ONLY;
    spi_config.datasize       = 8;
    spi_config.clock_polarity = SpiHandle::Config::ClockPolarity::LOW;
    spi_config.clock_phase    = SpiHandle::Config::ClockPhase::ONE_EDGE;
    spi_config.nss            = SpiHandle::Config::NSS::HARD_INPUT;
    spi_config.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_2;
    // SPI pin config
    spi_config.pin_config.sclk = {DSY_GPIOG, 11};
    spi_config.pin_config.miso = {DSY_GPIOX, 0};
    spi_config.pin_config.mosi = {DSY_GPIOB, 5};
    spi_config.pin_config.nss  = {DSY_GPIOG, 10};


	dmaSpi.Init(spi_config);


	dmaSpi.DmaReceive(rx_buffer,buffSize,nullptr,callbackFunction,&dmaSpi);

	//button setup
	



	hw.StartAudio(AudioCallback);

	while(1)
	{
		if(rx_buffer[0]==1)
		{
			hw.SetLed(true);
		}
		else
		{
			hw.SetLed(false);
		}
	}
}
