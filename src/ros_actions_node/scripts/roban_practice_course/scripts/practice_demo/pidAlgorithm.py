#!/usr/bin/env python
# -*- coding: utf-8 -*-

class PositionPID:
    kp, ki, kd = 0.0, 0.0, 0.0
    error, lastError = 0.0, 0.0
    errIntegral = 0.0
    integralMin, integralMax = -1000, 1000
    
    def integralLimit(self,integral):
        if integral > self.integralMax:
            integral = self.integralMax
        if integral < self.integralMin:
            integral = self.integralMin
        return integral

    def __init__(self,p=0.0,i=0.0,d=0.0):
        self.kp, self.ki, self.kd = p, i, d

    def setParameter(self,p,i,d):
        self.kp, self.ki, self.kd = p, i, d

    def setIntegralLimit(self,iMin,iMax):
        self.integralMin, self.integralMax = iMin, iMax

    def clearIntegral(self):
        self.errIntegral = 0

    def run(self,error):
        self.error = error
        self.errIntegral = self.errIntegral + self.error
        self.errIntegral = self.integralLimit(self.errIntegral)
        out = self.kp*self.error + self.ki*self.errIntegral + self.kd*(self.error-self.lastError)
        self.lastError = self.error
        return out
