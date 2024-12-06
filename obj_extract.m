function obj_def = obj_extract(image)
%{
ELG 5163 Machine Vision Challenge, 2024
Develop your "black box" solution below to process an input image and generate a scene contents descriptor

input:
    - image: raw RGB input image provided (.jpg)

output:
    - obj_def: an array with n+1 rows and 4 columns [(n+1)x4] containing the scene descriptor
	  [n          0      0      0
	   obj1_class obj1_X obj1_Y obj1_radius
	   obj2_class obj2_X obj2_Y obj2_radius
	   ...
  	   objn_class objn_X objn_Y objn_radius]
	   
	   Example with 3 objects (classes: red, green, blue) in sample image im5
	   	obj_def = {3,0,0,0; 'red',200,300,50; 'green',0,300,50; 'blue',-200,300,50}
		   
		Output "im5.dat" file content
		3,0,0,0
		red,200,300,50
		green,0,300,50
		blue,-200,300,50
%}


% *** BEGINNING OF YOUR BLACK BOX SOLUTION ***

color_classes = ["red", "green", "blue", "yellow", "orange", "pink", "cyan"];
color_hues = [345, 165, 215, 45, 15, 330, 195];

image = im2double(image);

hsv_image = rgb2hsv(image);
saturation = hsv_image(:,:,2);
iscolor = saturation > 0.2;

% Find circles
[centers,radii] = imfindcircles(iscolor,[15,100]);

assert(size(centers,1) >= 3, "ref points not found");
if size(centers,1) == 3
    % 3 ref points found, no objects
    obj_def= {0,0,0,0};
    return
end

% Find circle colors & remove uncolorful circles
colors = zeros(size(centers,1),1);
keep = zeros(size(centers,1),1,"logical");
for i = 1:size(centers,1)
    x_range = round(centers(i,1)) - round(radii(i)/2):round(centers(i,1)) + round(radii(i)/2);
    y_range = round(centers(i,2)) - round(radii(i)/2):round(centers(i,2)) + round(radii(i)/2);
    hsv = mean(hsv_image(y_range,x_range,:),[1,2]);
    keep(i) = hsv(2) > 0.2;
    hue = hsv(1)*360;
    [~,I] = min(abs(color_hues - hue));
    colors(i) = I;
end

% Remove uncolorful circles
centers = centers(keep,:);
radii = radii(keep);
colors = colors(keep);

% Sort by radius
[~,I] = sort(radii);
centers = centers(I,:);
radii = radii(I);
colors = colors(I);

% The 3 ref points should be much smaller than objects
assert(radii(3) < radii(4)/2, "biggest ref point too big, or smallest object too small");

ref_centers = centers(1:3,:);
ref_colors = colors(1:3);
obj_centers = centers(4:end,:);
obj_radii = radii(4:end);
obj_colors = colors(4:end);

orig_center = ref_centers(ref_colors == 1,:);
x_center = ref_centers(ref_colors == 3,:);
y_center = ref_centers(ref_colors == 2,:);

% Ref points should be red, green, blue
assert(size(orig_center,1) == 1 & size(x_center,1) == 1 & size(y_center,1) == 1, "wrong ref points");

% % Visualize
% figure(1);
% imshow(image);
% hold on;
% scatter(ref_centers(:,1),ref_centers(:,2),500,"x","black");
% for i = 1:size(obj_centers,1)
%     viscircles(obj_centers(i,:),obj_radii(i),"Color",hsv2rgb([color_hues(obj_colors(i))/360,1,1]));
%     text(obj_centers(i,1),obj_centers(i,2),color_classes(obj_colors(i)),"HorizontalAlignment","center");
% end
% hold off;

% Find the affine transformation
affine_tform = fitgeotform2d([orig_center;x_center;y_center],[0,0;100,0;0,100],"affine");

% Transform every point in the image
points = image_to_points(image);
points_tform = transformPointsForward(affine_tform,points(:,1:2)); % size=[n_points,5]

% Transform ref points and objects
ref_centers_tform = transformPointsForward(affine_tform,ref_centers);
obj_centers_tform = transformPointsForward(affine_tform,obj_centers);
obj_radii_tform = obj_radii * sqrt(affine_tform.T(1,1)^2 + affine_tform.T(2,1)^2);

% % Visualize
% figure(2);
% hold on;
% mask = rand(size(points,1),1) < 1;
% scatter(points_tform(mask,1),points_tform(mask,2),10,points(mask,3:5),".");
% scatter(ref_centers_tform(:,1),ref_centers_tform(:,2),500,"x","black");
% for i = 1:size(obj_centers_tform,1)
%     viscircles(obj_centers_tform(i,:),obj_radii_tform(i),"Color",hsv2rgb([color_hues(obj_colors(i))/360,1,1]));
%     text(obj_centers_tform(i,1),obj_centers_tform(i,2),color_classes(obj_colors(i)),"HorizontalAlignment","center");
% end
% hold off;
% axis equal;
% grid on;

% Find corners
corners = detectHarrisFeatures(rgb2gray(image),"MinQuality",0.3,"ROI",[100,100,size(image,2)-200,size(image,1)-200]);
corners_tform = transformPointsForward(affine_tform,corners.Location);

corners_grid = round(corners_tform, -2);

% Find the trandformation matrix
polynomial_tform = fitgeotform2d([corners_grid;0,0;100,0;0,100],[corners_tform;0,0;100,0;0,100],"polynomial",4);

% Transform every point in the image
points_tform_tform = transformPointsInverse(polynomial_tform,points_tform);

% Transform ref points and objects
ref_centers_tform_tform = transformPointsInverse(polynomial_tform,ref_centers_tform);
obj_centers_tform_tform = transformPointsInverse(polynomial_tform,obj_centers_tform);
corners_tform_tform = transformPointsInverse(polynomial_tform,corners_tform);

% % Visualize
% figure(3);
% hold on;
% mask = rand(size(points,1),1) < 1;
% scatter(points_tform_tform(mask,1),points_tform_tform(mask,2),10,points(mask,3:5),".");
% scatter(ref_centers_tform_tform(:,1),ref_centers_tform_tform(:,2),500,"x","black");
% scatter(corners_tform_tform(:,1),corners_tform_tform(:,2),100,".","red");
% for i = 1:size(obj_centers_tform_tform,1)
%     viscircles(obj_centers_tform_tform(i,:),obj_radii_tform(i),"Color",hsv2rgb([color_hues(obj_colors(i))/360,1,1]));
%     text(obj_centers_tform_tform(i,1),obj_centers_tform_tform(i,2),color_classes(obj_colors(i)),"HorizontalAlignment","center");
% end
% hold off;
% grid on;
% axis equal;

obj_def = cell(size(obj_centers_tform_tform,1),4);
obj_def(1,:) = {size(obj_centers_tform_tform,1), 0, 0, 0};
for i = 1:size(obj_centers_tform_tform)
    obj_def(i+1,:) = {color_classes(obj_colors(i)),round(obj_centers_tform_tform(i,1),2),round(obj_centers_tform_tform(i,2),2),round(obj_radii_tform(i),2)};
end

    function points = image_to_points(image)
        [xGrid, yGrid] = meshgrid(1:size(image, 2), 1:size(image, 1));
        
        R = image(:, :, 1);
        G = image(:, :, 2);
        B = image(:, :, 3);
        
        x = xGrid(:);
        y = yGrid(:);
        r = R(:);
        g = G(:);
        b = B(:);
        
        points = [x, y, r, g, b]; % As a matrix
        
    end

% *** END OF YOUR BLACK BOX SOLUTION ***


% Export obj_def to ".dat" text file
% writecell(obj_def,'im5.dat');
end
