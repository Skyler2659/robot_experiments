im=robo.image('top'); display(Image.fromarray(im[510:680,775:865]).resize((270,510))); stage2=tcp(); D=stage2@np.linalg.inv(stage1)
def stereo(a,b,z):
 return least_squares(lambda p:np.r_[proj(p)-a,proj((D@np.r_[p,1])[:3])-b],back(*a,z)).x
for a,b in [(mouth1,[812,529]),(cap1,[832,648])]:
 p=stereo(a,b,.23); print(p,(D@np.r_[p,1])[:3],proj(p)-a,proj((D@np.r_[p,1])[:3])-b)