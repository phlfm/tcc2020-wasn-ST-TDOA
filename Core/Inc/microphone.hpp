/* Class that handles microphone information
 * author: Pedro Henrique Lage Furtado de Mendonça - 2020
 *
 * Usage:
 *
 * This class holds a POINTER to spaceType[spaceDimension] (since it is a pointer, the data must be kept somewhere else
 *
 * It also holds a POINTER to a buffer in the format buffer[ch0, ch1, ch2, ..., ch_count, ch0, ch1, ch2, ...]
 * Calling Microphone[7] with Microphone.getChannel = 3 and Microphone.getChannelCount = 11 will access buffer[3+11*7]=buffer[80].
 *
 * This class employs basic bound checks and if violated enters an empty loop. (Careful that maybe optimizers will remove them)
 * */

#ifndef MIC_H
#define MIC_H



template <typename spaceType, unsigned char spaceDimension,  // space / position parameters
		unsigned int sampleBufferSize, typename sampleType, // sample parameters
		unsigned char channel, unsigned char channel_count> // channel parameters
class Microphone {
public:

	Microphone(spaceType* Position, sampleType* sampleBuffer) {
		this->Position = Position;
		this->sampleBuffer = sampleBuffer;
	} // CTOR

	~Microphone() {
		Position = nullptr; sampleBuffer = nullptr;
	} // DTOR



	// Returns Position[Dimension]. If Dimension is out of range, an infinite loop is caused.
	spaceType getPositionIndex(unsigned char Dimension) {if (Dimension > spaceDimension) {while(1){++Dimension;}}; return Position[Dimension];} // increment in dimension so optimizers don't remove it
	spaceType* getPosition() { return Position; }
	unsigned char getSpaceDimension() { return spaceDimension; }

	sampleType* getBuffer() { return sampleBuffer;}
	unsigned char getChannel() { return channel;}
	unsigned char getChannelCount() { return channel_count;}

	// Gets the buffers length relative to the channel and channel count (useful for iterations)
	unsigned int getBufferLength() {
		unsigned int mod = sampleBufferSize % channel_count;
		unsigned int maxIndex = (sampleBufferSize - mod) / channel_count; // This should yield an exact value
		if (mod >= (channel+1)) { ++maxIndex; }
		++maxIndex;
		return maxIndex;
	}

	sampleType &operator[](unsigned int i) {
		i = channel + i*channel_count; // Adapt i so the index is relative to the channel
        if( i >= sampleBufferSize ) { while(1) {++i;}}; // increment in loop so optimizers don't remove it
        return sampleBuffer[i];
     } // overload of [] operator


private:
	spaceType* Position = nullptr;
	sampleType* sampleBuffer = nullptr;

}; // class Microphone


#endif // MIC_H
