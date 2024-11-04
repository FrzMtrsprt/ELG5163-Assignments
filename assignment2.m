%% Part I - Effect of noise on image filtering

% Define images
image_helloworld.name = "HelloWorld";
image_eiffel.name = "Eiffel";
images = [image_helloworld,image_eiffel];
clear image_helloworld image_eiffel

% Define noises
noise_gaussian.name = "gaussian";
noise_gaussian.f = @(x) imnoise(x,"gaussian",0.01);
noise_saltpepper.name = "salt & pepper";
noise_saltpepper.f = @(x) imnoise(x,"salt & pepper", 0.03);
noises = [noise_gaussian,noise_saltpepper];
clear noise_gaussian noise_saltpepper

% Define filters
filter_gaussian.name = "gaussian";
filter_gaussian.f = @(x) imfilter(x,fspecial("gaussian",7,sqrt(2)));
filter_mean3.name = "mean3";
filter_mean3.f = @(x) imfilter(x,fspecial("average",3));
filter_mean7.name = "mean7";
filter_mean7.f = @(x) imfilter(x,fspecial("average",7));
filter_median.name = "median";
filter_median.f = @(x) medfilt2(x);
filters = [filter_gaussian,filter_mean3,filter_mean7,filter_median];
clear filter_gaussian filter_mean3 filter_mean7 filter_median

for i = 1:length(images)
    % Read i-th images
    images(i).m = imread(images(i).name+".jpg");
    images(i).noised = repmat({},length(noises));
    
    for j = 1:length(noises)
        % Apply j-th noises on i-th image
        images(i).noised(j).m = noises(j).f(images(i).m);
        imwrite(images(i).noised(j).m,images(i).name+"_"+noises(j).name+".png");
        images(i).noised(j).filtered = repmat({},length(filters));
        
        for k = 1:length(filters)
            % Apply k-th filter on j-th noised i-th image
            images(i).noised(j).filtered(k).m = filters(k).f(images(i).noised(j).m);
            imwrite(images(i).noised(j).filtered(k).m,images(i).name+"_"+noises(j).name+"_"+filters(k).name+".png");
        end
    end
end
clear i j k

%% Part II - Effect of noise and image sharpness or complexity on edge detection

% Define edge detectors
edges = ["sobel","prewitt","log","canny"];

for i = 1:length(images)
    
    for l = 1:length(edges)
        % Apply l-th edge detector on i-th image
        image = edge(images(i).m,edges(l));
        imwrite(image,images(i).name+"_"+edges(l)+".jpg");
        
        for j = 1:length(noises)
            % Apply l-th edge detector on j-th noised i-th image
            image = edge(images(i).noised(j).m,edges(l));
            imwrite(image,images(i).name+"_"+noises(j).name+"_"+edges(l)+".jpg");
        end
        
        % Apply l-th edge detector on 7x7 Gaussian filtered Gaussian noised i-th image
        image = edge(images(i).noised(1).filtered(1).m,edges(l));
        imwrite(image,images(i).name+"_"+noises(1).name+"_"+filters(1).name+"_"+edges(l)+".jpg");
        
        % Apply l-th edge detector on median filtered salt & pepper noised i-th image
        image = edge(images(i).noised(2).filtered(4).m,edges(l));
        imwrite(image,images(i).name+"_"+noises(2).name+"_"+filters(4).name+"_"+edges(l)+".jpg");
    end
end
clear i j k l image

%% Part III - Study of image segmentation

for i = 1:length(images)
    for threshold = 0.1:0.1:0.9
        % Apply segmentation with manual thresholding
        image = imbinarize(images(i).m,threshold);
        imwrite(image,images(i).name+"_manual_"+threshold+".jpg")
    end
    
    % Apply segmentation with adaptive thresholding
    image = imbinarize(images(i).m);
    imwrite(image,images(i).name+"_adaptive"+".jpg")
end
clear i threshold image
