
import math

kGain = 100.0
unknownVolume = 0.476 
tempDistance = 5.46829
m_lambda = 0.1386

result = kGain * unknownVolume * math.exp(-m_lambda * tempDistance)

print (result)