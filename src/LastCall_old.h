
#ifdef __cplusplus

#ifndef lastCall_old_h_
#define lastCall_old_h_
#include <arm_math.h>
#include <array>
#include <numeric>

//This class is used to estimate last call of a function that is called at a high frequency.
//It smoothes the time of the last call by means of history buffer that stores the time of the e.g. last 7 calls.
//Important: The class detects up to two outliers in the history and ignores them at the computation of the time of the last call
template <int HL>
struct HistoryOld{
    std::array<uint32_t,HL> data;
    bool valid=false;
    int32_t currentWriteIndex=0;
};
inline double toUInt32RangeOld(double x){
	constexpr double largestU32 = double(uint32_t(-1));
	constexpr double halfLargestU32 = 0.5*largestU32;
	if (x < -halfLargestU32){
		x += largestU32;
	}
	if (x > halfLargestU32){
		x -= largestU32;
	}
	return x;
}
template <int N>
class LastCallOld
{
    public:
        LastCallOld(){
            std::iota(_indices.begin(), _indices.end(), 0);
        }
        void reset(double expectedTimeStep){
            _expectedTimeStep=expectedTimeStep;
            _history.valid=false;
        }
        
        void addCall(uint32_t time){
            _history.data[_history.currentWriteIndex]=time;
            _history.currentWriteIndex++;
            if(_history.currentWriteIndex >=N){
                _history.currentWriteIndex=0;
            }
            if(!_history.valid){
                fillWithDefaultValues(_history);
            }
        }

        HistoryOld<N> getHistory() const {
            return _history;
        }
        uint32_t getLastDuration()const{
            int32_t upperIdx = (_history.currentWriteIndex +(N-1))%N;
            int32_t lowerIdx = (_history.currentWriteIndex +(N-2))%N;
            return _history.data[upperIdx] - _history.data[lowerIdx];
        }
        template <int O>
        double getLastCall(double expectedTimeStep) const {
            //We shift all data samples, so that the oldest sample becomes 0 and compute the center of mass of the shifted samples.
            uint64_t shiftedDataSum=0;
            const uint32_t oldest=_history.data[_history.currentWriteIndex];
            for (int32_t i =1; i< N; i ++){
                uint32_t idx=(_history.currentWriteIndex+i)%N;
                shiftedDataSum+=(_history.data[idx]-oldest);
            }
            double centerOfMass = double(shiftedDataSum)/N;
            // Serial.print("centerOfMass ");
            // Serial.println(centerOfMass);

            //now we want to detect outliers in the _history and shift a 'perfect' data sequence into the data samples:

            //assuming the 'perfect' data sequence is {0,expectedTimeStep,... (N-1)*expectedTimeStep}
            //the expected center of mass is:
            double centerOfMassExpected = double(N-1)*0.5*expectedTimeStep;
            double shift = centerOfMass - centerOfMassExpected;
            // Serial.print("centerOfMassExpected ");
            // Serial.println(centerOfMassExpected);
            
            std::array<double, O> largeErrors;
            std::fill(largeErrors.begin(), largeErrors.end(), 0.);
            std::array<int32_t, O> idxErrors;
            std::fill(idxErrors.begin(), idxErrors.end(), 0);         
            
            for (int32_t i =0; i< N; i ++){
                uint32_t idx=(_history.currentWriteIndex+i)%N;
                uint32_t dataS= _history.data[idx]-oldest;   //shift all data back, close to zero in order to prevent wrap around effects
                double error = abs(double(dataS - (shift +i*expectedTimeStep) ));
                for (size_t j =0; j< largeErrors.size(); j++){
                    if(error > largeErrors[j]){
                        for (size_t k = largeErrors.size()-1; k > j; k--){
                            largeErrors[k]=largeErrors[k-1];
                            idxErrors[k]=idxErrors[k-1];
                        }
                        largeErrors[j] = error;
                        idxErrors[j] = i;
                        break;
                    }
                }
            }
            if(largeErrors[0] == 0.){
                //the data seems to be perfect and we can just return the last sample
                uint32_t idx=(_history.currentWriteIndex+(N-1))%N;
                return double(_history.data[idx]);
            }
            //Now we compute the center of mass for a second time.
            //This time we don't use the two outliers
            {
                   
                std::array<int32_t, N> indices =_indices;
                for (int32_t idx : idxErrors){
                    indices[idx]=-1;
                }

                uint64_t shiftedDataSum=0;
                double shiftedDataSumExpected=0;
                for (int32_t i =1; i< N; i++){
                    if(indices[i] != -1){
                        uint32_t idx=(_history.currentWriteIndex+i)%N;
                        shiftedDataSum+=(_history.data[idx]-oldest);
                        shiftedDataSumExpected += i*expectedTimeStep;
                    }
                }
                
                double centerOfMass = double(shiftedDataSum)/(N-O); //-O because we skip two outliers
                double centerOfMassExpected = shiftedDataSumExpected/(N-O); //-O because we skip two outliers
                // Serial.print("centerOfMassExpected without outliers: ");
                // Serial.println(centerOfMassExpected);
                // Serial.print("centerOfMass without outliers: ");
                // Serial.println(centerOfMass);
                //(N-1)*expectedTimeStep is the last sample of the perfect sequence
                double shift = centerOfMass - centerOfMassExpected + oldest;
                // Serial.print("oldest: ");
                // Serial.println(oldest);
                // Serial.print("shift: ");
                // Serial.println(shift);
                // Serial.print("(N-1)*expectedTimeStep: ");
                // Serial.println((N-1)*expectedTimeStep);
                // Serial.print("(N-1)*expectedTimeStep+shift: ");
                // Serial.println((N-1)*expectedTimeStep+shift);
                double correctedSample =(N-1)*expectedTimeStep +shift; 
                constexpr double pow2_32= double(UINT32_MAX)+1.;
                if (correctedSample > UINT32_MAX){
                    correctedSample -= pow2_32;
                }
                else if(correctedSample < 0) {
                    correctedSample += pow2_32;
                }               
                return correctedSample;
            }
        }

    private:
        void fillWithDefaultValues(HistoryOld<N>& history) const {
            //filling with default values for convinience
            int32_t step = (N-1);   //equal to -1
            int32_t idx=(_history.currentWriteIndex +step)%N;
            uint32_t currentTime = _history.data[idx];
            idx=(idx+step)%N;
            for (int32_t i=1; i< N; i++){
                    history.data[idx]=currentTime - uint32_t(i*_expectedTimeStep +1);
                    idx=(idx+step)%N;
            }
            history.valid=true;
        }
        std::array<int32_t, N> _indices;
        HistoryOld<N> _history;
        double _expectedTimeStep=0;   //only used to initialize _history once        
};

#endif
#endif