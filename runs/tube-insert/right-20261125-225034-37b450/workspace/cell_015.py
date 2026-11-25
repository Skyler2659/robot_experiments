from PIL import ImageEnhance
def bright(box): display(ImageEnhance.Brightness(Image.fromarray(robo.image('top')).crop(box)).enhance(2.1))
P=tcp()@rel0; print('pred rim/tip',proj(P[:3,3]+P[:3,1]*.0525),proj(P[:3,3]-P[:3,1]*.046)); bright((380,685,455,770))