#include <SoftwareSerial.h>
#include <util\delay.h>
#define rxPin 8    // We use a non-existant pin as we are not interested in receiving data
#define txPin 2

#define outPWM 0
#define inPin 1
#define redLED 3 //red
#define cardDetectedLED 4 //yellow 

#define buffSize 208 //Must be divisible by 16

#define readNum 20 //Number of readings
#define minSuccessR 5 //Minimum number of successfull readings
#define maxCardNum 3  //Maximum number of cards detected for the readNum trials

unsigned char maxPulseSizeHigh = 7; //original code had 80. New value is 5
unsigned char maxPulseSizeLow = 5; //original code had 80. New value is 4

unsigned char pulsesData[buffSize];
byte multibit[buffSize / 8]; //Multibit buffer
byte singlebit[buffSize / 16];   //Singlebit buffer
byte finalData[7];  //Final data

unsigned long ManufacturerCode;
unsigned long RawNumber;
boolean readError = false;

SoftwareSerial serial = SoftwareSerial(rxPin, txPin);

void setup() {
	//Set PWM to 125kHz 50% dutycycle. See Attiny85 Datasheet to check PWM registers information
	pinMode(outPWM, OUTPUT);
	TCCR0A = 0;
	TCCR0B = 0;
	TCCR0A = 1 << COM0A0 | 1 << WGM01;
	TCCR0B = 1 << CS00;
	OCR0A = 0;

	//Start serial at 19200 bauds
	serial.begin(19200);

	//Set pin modes and initial conditions
	pinMode(inPin, INPUT);
	pinMode(redLED, OUTPUT);
	pinMode(cardDetectedLED, OUTPUT);
	digitalWrite(redLED, 0);
	digitalWrite(cardDetectedLED, 0);
}

void loop() {
	ManufacturerCode = 0;
	RawNumber = 0;

	//serial.print("\nReady...");

	// Wait for card to be detected (Wait for Initial high state)
	_delay_ms(100);
	while (digitalRead(inPin)) {
    digitalWrite(redLED, 0);
    digitalWrite(cardDetectedLED, 0);
    _delay_ms(50);
	}

	//If card detected turn on yellow led
	digitalWrite(cardDetectedLED, 1);


	byte numDetCard = 0;
	unsigned long detCardsRaw[maxCardNum] = { 0 };  //raw numbers detected for the same card (expected to be only one)
	byte numDetPerCard[maxCardNum] = { 0 };  //number of detections per detected card

	//when card detected try to read readnum times. for successfull reading, the raw number detected must be repeated minsuccessr times
	byte i = 0, k = 0;
	boolean successRead = false;
	readError = false;

	while (i < readNum) {
		//main read function
		RawNumber = 0;
		readCard();

		//check if there is a card
		if (readError)
			break;

//		serial.print("\nC:");
//		serial.println(RawNumber);

		//check if detected raw number is the same
		if (RawNumber != 0) {
			for (unsigned char j = 0; j < maxCardNum; j++) {
				if (RawNumber == detCardsRaw[j]) {
					numDetPerCard[j]++;
          serial.print(".");
					//serial.print(", n:");
					//serial.print(numdetpercard[j]);
					if (numDetPerCard[j] > minSuccessR) {
						i = readNum;
						successRead = true;
					}
					break;
				}
				else {
					if (numDetCard < maxCardNum) {
            serial.print("*");
						detCardsRaw[numDetCard] = RawNumber;
						numDetPerCard[numDetCard]++;
						numDetCard++;
						break;
					}
				}
			}
			i++;
		}
   else{
    serial.print("x");
   }
		k++;

		//exit after 120 not successfull read trials
		if (k > 120){
			serial.println("X");
			break;
		}
	}

	//if successfull reading turn on green led else turn on red led
	if (successRead) {				
		validCardDetected();
		byte checksum = checkSumCalculation(RawNumber);
		serial.print(RawNumber);
		serial.print("*");
		serial.println(checksum);
	}
	else
		invalidCardDetected();
}

//-------------------Main Card Read Function----------------
void readCard() {

	unsigned char pulseIdx = 0;
	unsigned char pulseTimer = 0;
	memset(pulsesData, 0, buffSize);

	//Get pulse durations (Take into account that the working frequency is set the same as the Timer 1,
	//thus too much processing here can incurr in incorrect pulse durations)
	while (pulseIdx < buffSize - 2) {
		while (digitalRead(inPin) == 1) {
			//_delay_us(7);
			pulseTimer++; // increment timer while input pin is high
      
      //Check if high pulse is too much time high, indicating that there is no present card
      if (pulseTimer > 250) {
        readError = true;
        return;
      }
		}
		pulsesData[pulseIdx] = pulseTimer;
		pulseTimer = 0;
		pulseIdx++;

		while (digitalRead(inPin) == 0) {
			//_delay_us(7);
			pulseTimer++; // increment timer while input pin is low			
		}
		pulsesData[pulseIdx] = pulseTimer;
		pulseTimer = 0;
		pulseIdx++;
	}
	
	//TEST
//	serial.print("\nPulses: ");
//	for (unsigned char i = 0; i < pulseIdx; i++) {
//		serial.print(pulsesData[i]);
//		serial.print("|");
//	}

	//Get Multibit representation
	getMultibit(pulseIdx);
	memset(pulsesData, 0, 1); //Clear pulses data variable

	//Print multibit char
//	serial.print("\nMultibit: ");
//	for (unsigned char i = 0; i < (buffSize / 8); i++) {
//		printCharAsBinary(multibit[i]);
//		serial.print("|");
//	}

	//Get Singlebit representation
	short stIdx = getSinglebit();

//Print singlebit char
//  serial.print("\nSB: ");
//  serial.print(stIdx);
//  serial.print(":");
//  for (int i = 0; i < buffSize / 16; i++) {
//    printCharAsBinary(singlebit[i]);
//    serial.print("|");
//  }
 
	if (stIdx == -1)
	  return;	

	//Get card data from start idx
	getCardData(stIdx, 0);

//	//Print final data
//	serial.print("\nCd: ");
//	for (int i = 0; i < 7; i++) {
//		printCharAsBinary(finalData[i]);
//		serial.print("|");
//	}

	if (!parityCheck()) {
		return;
	}
//	serial.print("\nPOK");

	//Get card numbers
	getRawNumber();
}

//Get the Multibit representation using the pulses duration stored in global variable
void getMultibit(unsigned char pulseIdx) {
	memset(multibit, 0, buffSize / 8);
	unsigned char j = 0;
	for (unsigned char i = 0; i < pulseIdx; i += 2) {
		if (pulsesData[i] < maxPulseSizeHigh) {
			setBitInByteArray(multibit, j, 1);
			j++;
		}
		else {
			setBitInByteArray(multibit, j, 1);
			j++;
			setBitInByteArray(multibit, j, 1);
			j++;
		}
		if (pulsesData[i + 1] < maxPulseSizeLow) {
			setBitInByteArray(multibit, j, 0);
			j++;
		}
		else {
			setBitInByteArray(multibit, j, 0);
			j++;
			setBitInByteArray(multibit, j, 0);
			j++;
		}
	}
}

//Get the Singlebit representation using the Multibit representation stored in global variable.
short getSinglebit() {
	byte currChar;
	byte nextChar;
	short j = 0;
	unsigned char nOnes = 0;
	short startIdx = -1;
	memset(singlebit, 0, buffSize / 16);
	for (short i = 0; i < buffSize / 2; i++) {
		currChar = getBitFromByteArray(multibit, j);
		nextChar = getBitFromByteArray(multibit, j + 1);
		j += 2;
		if (currChar == 1 && nextChar == 0) {
			setBitInByteArray(singlebit, i, 1);
			nOnes++;
			if (nOnes == 9 && startIdx == -1) {
				startIdx = i + 1;
			}
		}
		else {
			if (currChar != 0 || nextChar != 1)
				return -1;
			nOnes = 0;
			setBitInByteArray(singlebit, i, 0);
		}
	}
	return startIdx;
}

//Get the card data from the start idx found in the getSinglebit function.
void getCardData(unsigned char startIdx, unsigned char startCharIdx) {
	memset(finalData, 0, 7);
	byte currSBit;
	byte stIdx = startIdx;
	byte fDataIdx = 0;
	byte parityIdx = 40;
	for (unsigned char i = 0; i < 55; i++) {
		currSBit = getBitFromByteArray(singlebit, i + stIdx);

		if (i < 50) {
			if (((i + 1) % 5) != 0) {
				setBitInByteArray(finalData, fDataIdx, currSBit);
				fDataIdx++;
			}
			else {
				setBitInByteArray(finalData, parityIdx, currSBit);
				parityIdx++;
			}
		}
		else {
			setBitInByteArray(finalData, parityIdx, currSBit);
			parityIdx++;
		}
	}
}

//Check the data parity according EM4100 codification
boolean parityCheck() {

	byte parityRow = 0;
	byte parityCol[4] = { 0 };
	byte currByte;
	byte parityIdx = 40;

	//Loop through the 5 data bytes stored in finalData
	for (unsigned char i = 0; i < 40; i++) {

		currByte = getBitFromByteArray(finalData, i);
		parityRow ^= currByte;
		parityCol[i % 4] ^= currByte;

		//Check row parity
		if ((i + 1) % 4 == 0) {
			currByte = getBitFromByteArray(finalData, parityIdx);
			if (parityRow != currByte)
				return false;
			parityRow = 0;
			parityIdx++;
		}
	}
	//Check column parity
	for (unsigned char i = 0; i < 4; i++) {
		currByte = getBitFromByteArray(finalData, parityIdx);
		parityIdx++;
		if (parityCol[i] != currByte)
			return false;
	}
	return true;
}

//Get Rawnumber and manufacturer code
void getRawNumber() {
	ManufacturerCode = finalData[0];
	for (char i = 1; i < 5; i++) {
		RawNumber |= ((unsigned long)finalData[i] << (8 * (4 - i)));
	}
}

//Calculate checksum
byte checkSumCalculation(unsigned long RawNumber) {
	char rawData[8];
	byte nChars = sprintf(rawData, "%lu", RawNumber);
	byte checksum;
	for (unsigned char i = 0; i < nChars; i++) {
		checksum ^= (byte)rawData[i];
	}
	return checksum;
}

void validCardDetected() {
	for (char i = 0; i < 10; i++) {
		digitalWrite(redLED, 1);
		_delay_ms(100);
		digitalWrite(redLED, 0);
		_delay_ms(100);
	}
}

void invalidCardDetected() {
	digitalWrite(redLED, 1);
	_delay_ms(1200);
	digitalWrite(redLED, 0);
}

byte getBitFromByteArray(byte *A, byte idx) {
	byte b = 0;
	byte aIdx = idx / 8;
	byte aBitIdx = idx % 8;
	b = (A[aIdx] >> ((7 - aBitIdx)) & 1);
	return b;
}

void setBitInByteArray(byte *A, byte idx, byte val) {
	byte aIdx = idx / 8;
	byte aBitIdx = idx % 8;
	A[aIdx] |= (val << (7 - aBitIdx));
}

void printCharAsBinary(char a) {
  for (char k = 7; k >= 0; k--) {
    serial.print((a >> k) & 1);
  }
}

//void printLongAsBinary(unsigned long a) {
//  for (char k = 31; k >= 0; k--) {
//    Serial.print((a >> k) & 1);
//  }
//}





