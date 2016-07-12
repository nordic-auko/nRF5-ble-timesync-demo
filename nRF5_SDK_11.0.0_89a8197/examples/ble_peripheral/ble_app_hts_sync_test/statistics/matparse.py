import sys,os
import scipy.io as sio
import numpy as np

def parse_mat(filename):

    print "Opening mat file: " + filename
    
    trace = sio.loadmat(sys.argv[1])
     
    ch0  = trace['digital_channel_0']
    ch1  = trace['digital_channel_1']
    freq = trace['digital_sample_rate_hz']
    samples = trace['num_samples_digital']
     
    flips = min(len(ch0),len(ch1))
    time0 = 0.0
    time1 = 0.0
    diffs = np.zeros(flips)

    for i in xrange(0, flips):
        time0 += ch0[i][0]
        time1 += ch1[i][0]
        diff = abs(time0-time1)

        if (diff > 10000.0):
            # Sanity check
            print "Missed sample. Assuming remainder of trace is bad"
            diffs = diffs[0:i]
            break
        else:
            diffs[i] = diff
    
    return diffs, freq[0][0], time0, time1

def parse_vcd(filename):
    # Assuming hard coded format:
    # $timescale 1 ns $end
    # $scope module top $end
    # $var wire 1 ! CH0_name $end
    # $var wire 1 " CH1_name $end
    # $upscope $end
    # $enddefinitions $end

    print "Opening vcd file: " + filename
    fin = open(filename)
    
    if "timescale 1 ns" not in fin.readline():
        print "Could not parse VCD: "
        exit(1)
    if "$scope module top $end" not in fin.readline():
        print "Could not parse VCD"
        exit(1)
    if "$var wire 1 !" not in fin.readline():
        print "Could not parse VCD"
        exit(1)
    if "$var wire 1 \"" not in fin.readline():
        print "Could not parse VCD"
        exit(1)
    if "$upscope $end" not in fin.readline():
        print "Could not parse VCD"
        exit(1)
    if "$enddefinitions $end" not in fin.readline():
        print "Could not parse VCD"
        exit(1)
    
    data = []
    
    for line in fin:
        data.append(line)
    
    ch0_flips = 0
    ch1_flips = 0
    
    for s in data:
        if "!" in s:
            ch0_flips += 1
        if "\"" in s:
            ch1_flips += 1

    flips = min(ch0_flips, ch1_flips)
    ch0   = np.zeros(ch0_flips)
    ch1   = np.zeros(ch1_flips)
    i0    = 0
    i1    = 0
    
    time = 0.0
    for s in data:
        if "#" in s:
            time = float(s[1::])
        if "!" in s:
            ch0[i0] = time
            i0     += 1
        if "\"" in s:
            ch1[i1] = time
            i1     += 1
    
    diffs = np.zeros(flips)
    freq  = 1000000000.0
    
    for i in xrange(0, flips):
        diff = abs(ch0[i]-ch1[i])

        if (diff > 100000.0):
            # Sanity check
            print "Missed sample. Assuming remainder of trace is bad"
            diffs = diffs[0:i]
            break
        else:
            diffs[i] = diff
    
    return diffs, freq, ch0[-1] - ch0[0], ch1[-1] - ch1[0]
    
if len(sys.argv) < 2:
    print "Usage: python " + sys.argv[0] + " trace.mat/vcd"
    exit(1)

if ".mat" in sys.argv[1]:
    diffs, freq, time0, time1 = parse_mat(sys.argv[1])
elif ".vcd" in sys.argv[1]:
    diffs, freq, time0, time1 = parse_vcd(sys.argv[1])
else:
    print "Unknown file format"
    exit(1)
        
print str(freq) + "  Hz sampling freq"
print str(len(diffs)) + " toggles found within sanity range"
print "Total time in channel 0: " + str((time0 / freq)) + " s"
print "Total time in channel 1: " + str((time1 / freq)) + " s"

max_val = 0
max_idx = 0
for i in xrange(0, len(diffs)):
    if diffs[i] > max_val:
        max_val = diffs[i]
        max_idx = i

print "Largest difference: " + str((max_val / freq) * 1000000000.0) + " ns at toggle #" + str(max_idx) + " (" + str(max_idx * (65536.0 / 16000000.0)) + " seconds)"

mean = (np.mean(diffs) / freq) * 1000000000.0
std  = (np.std(diffs) / freq) * 1000000000.0

print "Mean difference across " + str(len((diffs))) + " toggles = " + str(mean) + " ns"
print "Standard deviation across " + str(len((diffs))) + " toggles = " + str(std) + " ns"
