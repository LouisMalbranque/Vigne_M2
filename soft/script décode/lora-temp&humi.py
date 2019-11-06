#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      louis
#
# Created:     04/11/2019
# Copyright:   (c) louis 2019
# Licence:     <your licence>
#-------------------------------------------------------------------------------


def main():
	while (1):
		btemp = int(input("Temperature: "), 16)
		temp = int(btemp*70/255)-30
		print(temp)

		bhumi = int(input("Humidite: "), 16)
		humi = int(bhumi*100/255)
		print(humi)





if __name__ == '__main__':
    main()
