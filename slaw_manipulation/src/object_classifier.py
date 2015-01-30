#!/usr/bin/env python

import numpy as np
import cv2
import StringIO
import sys

class Classifier():
    def __init__(self, fname, test = False):
        """
        """
        data_without_header = StringIO.StringIO()
        self.objects = None
        counter = 0
        with open(fname) as f:
            save_rest = False
            for line in f:
                if save_rest:
                    data_without_header.write(line)
                elif 'DATA' in line:
                    save_rest = True
                elif 'class' in line:
                    self.objects = line[line.index('{')+1:line.index("}")].split(', ')
                elif 'ATTRIBUTE' in line:
                    counter += 1
        #print objects, counter
        data_without_header.seek(0)
        #for l in data_without_header:
        #    print l
        data= np.loadtxt(data_without_header, dtype= 'float32', delimiter = ',',
                         converters = {counter: lambda label: self.objects.index(label) })

        train_data, labels = np.hsplit(data,[counter])
        #print labels
        data_without_header.close()
        self.knn = cv2.KNearest()
        self.knn.train(train_data, labels)
        #print data

        if test:
            import random
            for i in xrange(100):
                
                num = random.randint(0, len(train_data)-1)
                res = self.find_label(train_data[num])
                print res[1], res[2]
            
    def find_label(self, features, k = 5):
        #print features
        ret, result, neighbours, dist = self.knn.find_nearest(np.array([features], dtype='float32'), k)
        return ret, self.objects[int(result[0])], neighbours[0], dist[0]
        
if __name__ == '__main__':
    fname = "./"
    if len(sys.argv) > 1:
        fname = "./" + sys.argv[1]
    else:
        print "usage object_classifer.py <filename>"
        sys.exit()

    classifier = Classifier(fname, test = True)