# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                                                                 #
# Copyright (c) 2018, William C. Lenthe                                           #
# All rights reserved.                                                            #
#                                                                                 #
# Redistribution and use in source and binary forms, with or without              #
# modification, are permitted provided that the following conditions are met:     #
#                                                                                 #
# 1. Redistributions of source code must retain the above copyright notice, this  #
#    list of conditions and the following disclaimer.                             #
#                                                                                 #
# 2. Redistributions in binary form must reproduce the above copyright notice,    #
#    this list of conditions and the following disclaimer in the documentation    #
#    and/or other materials provided with the distribution.                       #
#                                                                                 #
# 3. Neither the name of the copyright holder nor the names of its                #
#    contributors may be used to endorse or promote products derived from         #
#    this software without specific prior written permission.                     #
#                                                                                 #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"     #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE       #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  #
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE    #
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL      #
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR      #
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,   #
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.            #
#                                                                                 #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import numpy as np
import matplotlib.pyplot as plt

# open shifts text file
filename = 'corrected/shifts.txt'
data = np.genfromtxt(filename, loose = True, delimiter = '\t', skip_header = 1)
names = np.loadtxt(filename, skiprows = 1, dtype = 'S', usecols = (0,))
names = [n.decode('utf-8') for n in names] # bytes -> string

# scatter plot shifts
colors = []
for i in range(len(names)):
	# each row is name, b, c, shifts...
	b = data[i][1]
	c = data[i][2]
	shifts = data[i][3:]
	p = plt.plot(np.linspace(0,len(shifts)-1,len(shifts)), shifts, marker = 'o', linestyle = '')
	colors.append(p[0].get_color()) # save color of scatter plot

# line plot fits
for i in range(len(names)):
	# each row is name, b, c, shifts...
	b = data[i][1]
	c = data[i][2]
	vMax = len(data[i])-4
	x = np.linspace(0,vMax)
	y = 1.0 - np.exp(-b * np.exp(-c * x))
	plt.plot(x,y, color = colors[i])

plt.xlabel('Sample Number')
plt.ylabel('Shift (Pixels)')
plt.legend(names)
plt.savefig('shifts.pdf')