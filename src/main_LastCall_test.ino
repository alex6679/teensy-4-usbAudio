#include "Arduino.h"
#include <util/LastCall.h>
#include "LastCall_old.h"

void evalOldLastCall(const std::vector<uint32_t>& times, uint32_t duration){

  constexpr uint32_t N=50;
  constexpr uint32_t O=20;
  LastCallOld<N> lastCall;
  lastCall.reset(duration);
  double maxError = 0;
  double meanError = 0.;
  uint32_t clockCountBefore, clockCountAfter;
  uint32_t clockSum = 0;
  uint32_t errorCount = 0;
  double stdDev=0.;
  for (uint32_t i = 0; i < times.size(); i++){

	  clockCountBefore = ARM_DWT_CYCCNT;
    lastCall.addCall(times[i]);
	  clockCountAfter = ARM_DWT_CYCCNT;
    clockSum += (clockCountAfter-clockCountBefore);

    if (i%1 == 0){
      uint32_t expected = i*duration;

	    clockCountBefore = ARM_DWT_CYCCNT;
      double lastCallComputed = lastCall.getLastCall<O>(duration);
	    clockCountAfter = ARM_DWT_CYCCNT;
      clockSum += (clockCountAfter-clockCountBefore);
      double error=  toInt32Range(expected-lastCallComputed);
      if (i > N && std::abs(error) > maxError){
        maxError = std::abs(error);
      }
      meanError+=error;
      errorCount++;
      stdDev +=error*error;
    }
  }  
  meanError/=errorCount;
  
  double toMilliSec = 1e3/F_CPU_ACTUAL;
  Serial.print("ellapsed time old: ");
  Serial.println(double(clockSum)*toMilliSec);
  
  Serial.print("mean error: ");
  Serial.println(meanError, 4);

  Serial.print("maxError: ");
  Serial.println(maxError, 4);

  Serial.print("Standard deviation last call old: ");
  Serial.println(std::sqrt(stdDev/errorCount), 4);
}


void evalLastCall(const std::vector<uint32_t>& times, uint32_t duration){

  constexpr uint32_t N=50;
  constexpr uint32_t O=20;
  LastCall<N> lastCall;
  lastCall.reset(duration);
  double maxError = 0;
  double meanError = 0.;
  uint32_t clockCountBefore, clockCountAfter;
  uint32_t clockSum = 0;
  uint32_t errorCount = 0;
  double stdDev=0.;
  for (uint32_t i = 0; i < times.size(); i++){

	  clockCountBefore = ARM_DWT_CYCCNT;
    lastCall.addCall(times[i]);
	  clockCountAfter = ARM_DWT_CYCCNT;
    clockSum += (clockCountAfter-clockCountBefore);

    if (i%1 == 0){
      uint32_t expected = i*duration;

	    clockCountBefore = ARM_DWT_CYCCNT;
      double lastCallComputed = lastCall.getLastCall<O>();
	    clockCountAfter = ARM_DWT_CYCCNT;
      clockSum += (clockCountAfter-clockCountBefore);
      double error=  toInt32Range(expected-lastCallComputed);
      if (i > N && std::abs(error) > maxError){
        maxError = std::abs(error);
      }
      meanError+=error;
      errorCount++;      
      stdDev +=error*error;
    }
  }
  
  meanError/=errorCount;
  
  double toMilliSec = 1e3/F_CPU_ACTUAL;
  Serial.print("ellapsed time new: ");
  Serial.println(double(clockSum)*toMilliSec);
  
  Serial.print("mean error: ");
  Serial.println(meanError, 4);

  Serial.print("maxError: ");
  Serial.println(maxError, 4);

  Serial.print("Standard deviation last call new: ");
  Serial.println(std::sqrt(stdDev/errorCount), 4);

}

void addRandomNoise(uint32_t amplitude, uint32_t& x, double& meanNoise){
  const uint32_t noise=1+uint32_t(random(2*amplitude-1));
  if (noise < amplitude){
      x -=noise;
      meanNoise -=(double)noise;
  }
  else {
      x +=(noise-amplitude);
      meanNoise+=(noise-amplitude);
  }
}

void setup() {
  while(!Serial){}

  uint32_t duration = 2000;
  uint32_t noiseAmplitude = 15;
  uint32_t outlierAmplitude = 30;
  
  uint32_t n = 10000;
  std::vector<uint32_t> times(n);
  double meanNoise=0;
  double stdDev=0.;

  for (uint32_t i = 0; i < n; i++){
    uint32_t d = i*duration;
    addRandomNoise(noiseAmplitude, d, meanNoise);
    //add outliers
    int o = random(8);
    if (o==0){
      double dummy;
      addRandomNoise(outlierAmplitude, d, dummy);
    }
    times[i]=d;
    double e = (double)d- (double)i*duration;
    stdDev +=e*e;
  }
  Serial.print("Mean noise: ");
  Serial.println(meanNoise/n);
  Serial.print("Standard deviation raw: ");
  Serial.println(std::sqrt(stdDev/n), 4);
  Serial.println();
  Serial.println("Evaluating the old LastCall.");
  evalOldLastCall(times, duration);
  Serial.println();
  Serial.println("Evaluating the new LastCall.");
  evalLastCall(times, duration);
}
void loop() {

}
