#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      louis
#
# Created:     28/10/2019
# Copyright:   (c) louis 2019
# Licence:     <your licence>
#-------------------------------------------------------------------------------

def main():

	while(1):
		data = int(input("Valeur"), 16)
		humidity = (data & 0b00111111111111110000000000000000) >> 16

		humidity = (humidity/(2**14-2))*100



		temperature = (data & 0b1111111111111100) >> 2
		temperature = (temperature/(2**14-2))*165-40

		print("température : " + str(temperature) + ", humidité : " + str(humidity))









if __name__ == '__main__':
    main()
