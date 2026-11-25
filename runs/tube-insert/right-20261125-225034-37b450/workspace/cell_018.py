uvR=(870.5,459.5); uvB=(881,599); L=.09857
base=lambda z: bp(*uvB,z); top=lambda z: bp(*uvR,z+L)
diff=top(0)-base(0); slope=top(1)-base(1)-diff; hfit=-diff[0]/slope[0]
print('z tip fit',hfit,'xy tip',base(hfit),'xy rim',top(hfit),'target',np.array(hole)); print([(z,base(z)[:2],top(z)[:2]) for z in [.164,.17,.175,.18,.185]])
print('actual tcp',tcp()[:3,3])