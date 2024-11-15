#!/usr/bin/env python  

__author__ = 'Raphael Leber'

from abc import ABC, abstractmethod


class Vehicle:

    @abstractmethod
    def model(self, speed, guidon, dt):

        pass        


    @abstractmethod
    def toPoint(self, x, y, eps=0.5):
        """ Move to a point (x, y)

            Parameters:
            x (int): x coordinate to reach in the map
            y (int): y coordinate to reach in the map
            eps (float): epsilon below which we consider the goal is reached

            Returns:
            float: Returning throttle

        """    
        pass 


    @abstractmethod
    def turn(self, theta):

        pass    