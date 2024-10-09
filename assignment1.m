%% Part I - Create a virtual scene

% Define the object planes' transformations
Rp1 = [
    0.866,0,-0.5,0;
    0,1,0,0;
    0.5,0, 0.866,1700;
    0,0,0,1;];
Rp2 = [
    0.9659,0,0.2588,-1830;
    0 ,1,0,0;
    -0.2588,0,0.9659, 1460;
    0,0,0,1;];
Rp3 = [
    0.5,0,0.866,1366;
    0,1,0,0;
    -0.866,0,0.5,1334;
    0,0,0,1;];

% Define objects' properties
step = 2;
% Center of Plane1
object.x = -499:step:500;
object.y = -499:step:500;
object.R = Rp1;
object.color = [1;0;1]; % Magenta
objects = object;

% Top of Plane1
object.x = -999:step:1000;
object.y = -999:step:-500;
object.R = Rp1;
object.color = [1;1;0]; % Yellow
objects(end+1) = object;

% Bottom of Plane1
object.x = -999:step:1000;
object.y = 501:step:1000;
object.R = Rp1;
object.color = [1;1;0]; % Yellow
objects(end+1) = object;

% Left of Plane1
object.x = -999:step:-500;
object.y = -499:step:500;
object.R = Rp1;
object.color = [1;1;0]; % Yellow
objects(end+1) = object;

% Right of Plane1
object.x = 501:step:1000;
object.y = -499:step:500;
object.R = Rp1;
object.color = [1;1;0]; % Yellow
objects(end+1) = object;

% Top of Plane2
object.x = -999:step:1000;
object.y = -999:step:0;
object.R = Rp2;
object.color = [0;0;1]; % Blue
objects(end+1) = object;

% Bottom of Plane2
object.x = -999:step:1000;
object.y = 1:step:1000;
object.R = Rp2;
object.color = [0;1;0]; % Green
objects(end+1) = object;

% Left of Plane3
object.x = -999:step:-334;
object.y = -999:step:1000;
object.R = Rp3;
object.color = [1;0;0]; % Red
objects(end+1) = object;

% Middle of Plane3
object.x = -333:step:332;
object.y = -999:step:1000;
object.R = Rp3;
object.color = [0;1;0]; % Green
objects(end+1) = object;

% Right of Plane3
object.x = 333:step:1000;
object.y = -999:step:1000;
object.R = Rp3;
object.color = [0;0;1]; % Blue
objects(end+1) = object;

clear Rp1 Rp2 Rp3 object step

% Calculate the total number of points
for i = 1:size(objects,2)
    objects(i).n = size(objects(i).x,2)*size(objects(i).y,2);
end
clear i
n_sum = sum([objects.n]);

% Sample and transform the objects
point_cloud = zeros(7, n_sum);
pos = 1;
for obj = objects
    [X,Y] = meshgrid(obj.x,obj.y);
    X = X(:)'; % Flatten and transpose
    Y = Y(:)';
    points = obj.R*[X; Y; zeros(1,obj.n); ones(1,obj.n)];
    
    color = repmat(obj.color, 1, obj.n);
    point_cloud(1:4, pos:pos+obj.n-1) = points;
    point_cloud(5:7, pos:pos+obj.n-1) = color;
    pos = pos+obj.n;
end
clear objects obj X Y points color pos

% % Plot the planes in 3D
% scatter3(point_cloud(1,:),point_cloud(2,:),point_cloud(3,:),1,point_cloud(5:7,:).',"filled");
% xlim([-3000,3000])
% ylim([-1000,1000])
% zlim([0 2500])
% xlabel("x")
% ylabel("y")
% zlabel("z")
% daspect([1 1 1])

%% Part II - Develop a non-inverting pinhole camera simulator and study the influence of its internal parameters

% Define the camera intrinsics
cam1.dim = [640,640,3]; % y x c
cam1.res = [640/8.5,640/8.5];
cam1.R = [
    1,0,0,0;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1;];

% Implementation of "take_photo" is at the end of this file
image = take_photo(point_cloud,cam1,4);
imwrite(image,"IM_Rcam1_f4.jpg");
image = take_photo(point_cloud,cam1,2);
imwrite(image,"IM_Rcam1_f2.jpg");
image = take_photo(point_cloud,cam1,7);
imwrite(image,"IM_Rcam1_f7.jpg");

%% Part III - Study the influence of the camera viewpoint on image formation

cam2.dim = [640,640,3]; % y x c
cam2.res = [640/8.5,640/8.5];
cam2.R = [
    0.866,0,-0.5 ,1400;
    0,1,0,0;
    0.5,0,0.866,-500;
    0,0,0,1;];

image = take_photo(point_cloud,cam2,4);
imwrite(image,"IM_Rcam2_f4.jpg");

cam3.dim = [640,640,3]; % y x c
cam3.res = [640/8.5,640/8.5];
cam3.R = [
    0.7071,0,0.7071,-2000;
    0,1,0,0;
    -0.7071,0,0.7071,0;
    0,0,0,1;];

image = take_photo(point_cloud,cam3,4);
imwrite(image,"IM_Rcam3_f4.jpg");

function image = take_photo(point_cloud,cam,f)

% Transform the points into the camera world
cam_points = inv(cam.R)*point_cloud(1:4,:);
% Scale the points to the image plane
w = cam_points(3,:)/f;
pos_x = cam_points(1,:)./w;
pos_y = cam_points(2,:)./w;

% Plot the image plane
% scatter(pos_x,pos_y,1,point_cloud(5:7,:).')

% Find the pixel position of each point
image = ones(cam.dim)*0.5;
pix_x = ceil(pos_x*cam.res(1)) + cam.dim(1)/2;
pix_y = ceil(pos_y*cam.res(2)) + cam.dim(2)/2;

% Find points that are in the image
indices = find(pix_x > 1 & pix_x < cam.dim(1) & pix_y > 1 & pix_y < cam.dim(2));

% Paint the image
for i = indices
    image(pix_y(i),pix_x(i),1) = point_cloud(5,i);
    image(pix_y(i),pix_x(i),2) = point_cloud(6,i);
    image(pix_y(i),pix_x(i),3) = point_cloud(7,i);
end
end
