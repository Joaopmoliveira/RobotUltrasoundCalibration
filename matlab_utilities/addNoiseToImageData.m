function data = addNoiseToImageData(data,noise_ratio)

data.W =data.W+ noise_ratio*2*(rand(3,size(data.W,2),size(data.W,3))-repmat(0.5,[3 size(data.W,2) size(data.W,3)]));

end