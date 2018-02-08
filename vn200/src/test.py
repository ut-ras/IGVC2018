#!/usr/bin/python

import random

eps = 0.1

mag_2dlist = [[] for i in range(3)]
loop = 1
counter = [0,0,0]
adding = 1
finish = 1000

mag = [0,0,0]
mag_max = [0,0,0]
mag_min = [0,0,0]
previous_max = [0,0,0]
previous_min = [0,0,0]


while(loop):
	for j in range(3):
		mag[j] = random.uniform(-1,5)

	for j in range(3):
		mag_2dlist[j].append(mag[j])

		mag_max[j] = max(mag_2dlist[j])
		mag_min[j] = min(mag_2dlist[j])

		#eps_max.insert(j,mag_2dlist[j][-1] - mag_max[j])
		#eps_min.insert(j,mag_2dlist[j][-1] - mag_min[j])

		#if abs(eps_max[0]) < eps and abs(eps_max[1]) < eps and abs(eps+max[2]) < eps and abs(eps_min[0]) < eps and abs(eps_min[1]) < eps and abs(eps_min[2]) < eps:
		#if abs(eps_max[0]) < eps:
		#	loop = 0

		#print(mag_2dlist)
	#print(mag_max[2] - previous_max[2])

	for j in range(3):
		if(mag_max[j] - previous_max[j]) == 0.0:
			counter[j] = counter[j] + 1
		elif(mag_max[j] - previous_max[j] != 0.0):
			counter[j] = 0
		previous_max[j] = mag_max[j]

	#print(counter)

	if counter[0] > 1000 and counter[1] > 1000 and counter[2] > 1000:
		loop = 0

print(mag_max)



#print mag_max
#print mag_min

#def func():
	#sample = 20
	#x = 0
	#for i in range(sample):
		#x = x + 1
		#print(x)

# 	eps = 0.1

# 	mag_2dlist = [[] for i in range(3)]
# 	loop = 1

# 	mag = []
# 	mag.insert(0,random.uniform(-1,5))
# 	mag.insert(1,random.uniform(-1,5))
# 	mag.insert(2,random.uniform(-1,5))

# 	mag_max = []
# 	mag_min = []
# 	eps_max = []
# 	eps_min = []

# 	while(loop):
# 		for j in range(3):
# 			mag_2dlist[j].append(mag[j])

# 			mag_max.insert(j,max(mag_2dlist[j]))
# 			mag_min.insert(j,min(mag_2dlist[j]))

# 			eps_max.insert(j,mag_2dlist[j][-1] - mag_max[j])
# 			eps_min.insert(j,mag_2dlist[j][-1] - mag_min[j])

# 			if abs(eps_max[0]) < eps and abs(eps_max[1]) < eps and abs(eps+max[2]) < eps and abs(eps_min[0]) < eps and abs(eps_min[1]) < eps and abs(eps_min[2]) < eps:
# 				loop = 0

# 			print(mag_max)

# 	print mag_max
# 	print mag_min

# 	#x = random.uniform(-1.0,5.0)

# 	#print(x)


# if __name__ == '__main__':
# 	try:
# 		func()
# 	except:
# 		pass