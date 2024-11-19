% Define the camera intrinsics
cam.dim = [640,640]; % y x
cam.res = [640/8.5,640/8.5];
cam.R = [
    1,0,0,0;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1;];

%% Part I (Output type 1 - Textured point cloud):
point_cloud = construct_scene(20);

% Capture RGB-D image with camera
image = capture_rgbd_image(point_cloud,cam,2);

% Convert RGB-D image to point cloud
captured_points = rgbd_to_point_cloud(image,cam,2);

% Display & save the point cloud to image
figure("visible","off");
ax = pcshow(pointCloud(captured_points(:,1:3),Color=captured_points(:,4:6)));
ax.XLabel.String = "X";
ax.YLabel.String = "Y";
ax.ZLabel.String = "Z";
ax.Color = "w";
ax.XColor = 'k';
ax.YColor = 'k';
ax.ZColor = 'k';
ax.View = [0,-90];
exportgraphics(ax,"pointcloud_front.pdf","ContentType","vector")
ax.View = [0,0];
exportgraphics(ax,"pointcloud_top.pdf","ContentType","vector")
clear ax;

%% Part II (Output type 2 - Cartesian depth map):
point_cloud = construct_scene(2);

% Capture RGB-D image with camera
image = capture_rgbd_image(point_cloud,cam,2);

% Convert RGB-D image to Cartesian depth map
depth_map = rgbd_to_cartesian(image,cam,2);

% Normalize the image
has_object = depth_map < Inf;
depth_map(has_object) = (depth_map(has_object) - min(depth_map(has_object)))/(max(depth_map(has_object)) - min(depth_map(has_object)))*0.5+0.5;
depth_map(~has_object) = 0;

% Save the image & the posterized image
imwrite(depth_map,"cartesian.png")
depth_map = round(depth_map,1);
imwrite(depth_map,"cartesian_posterized.png")

%% Part III (Output type 3 - Spherical depth map):
point_cloud = construct_scene(1);

% Capture RGB-D image with camera
image = capture_rgbd_image(point_cloud,cam,2);

% Convert RGB-D image to spherical depth map
depth_map = rgbd_to_spherical(image);

% Normalize the image
has_object = depth_map < Inf;
depth_map(has_object) = (depth_map(has_object) - min(depth_map(has_object)))/(max(depth_map(has_object)) - min(depth_map(has_object)))*0.5+0.5;
depth_map(~has_object) = 0;

% Save the image & the posterized image
imwrite(depth_map,"spherical.png")
depth_map = round(depth_map,1);
imwrite(depth_map,"spherical_posterized.png")

%% Function for constructing the at a given step
function point_cloud = construct_scene(step)
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

end

%% Function for capturing the RGB-D image
function image = capture_rgbd_image(point_cloud,cam,f)

% Transform the points into the camera world
cam_points = inv(cam.R)*point_cloud(1:4,:);
% Scale the points to the image plane
w = cam_points(3,:)/f;
pos_x = cam_points(1,:)./w;
pos_y = cam_points(2,:)./w;

% Find the pixel position of each point
pix_x = ceil(pos_x*cam.res(1)) + cam.dim(1)/2;
pix_y = ceil(pos_y*cam.res(2)) + cam.dim(2)/2;

% Find points that are in the image plane
indices = find(pix_x > 1 & pix_x < cam.dim(1) & pix_y > 1 & pix_y < cam.dim(2));

% capture the RGB-D image
image = Inf(cam.dim(1),cam.dim(2),4);
for i = indices
    image(pix_y(i),pix_x(i),1) = point_cloud(5,i);
    image(pix_y(i),pix_x(i),2) = point_cloud(6,i);
    image(pix_y(i),pix_x(i),3) = point_cloud(7,i);
    image(pix_y(i),pix_x(i),4) = sqrt(point_cloud(1,i)^2+point_cloud(2,i)^2+point_cloud(3,i)^2);
end

end

%% Function for converting RGB-D image to point cloud
function rep = rgbd_to_point_cloud(image,cam,f)
rep = Inf(cam.dim(1),cam.dim(2),6);
% Calculate the coordinates of pixel positions.
pos_x = ((1-cam.dim(1)/2):(cam.dim(1)/2))/cam.res(1);
pos_y = ((1-cam.dim(2)/2):(cam.dim(2)/2))/cam.res(2);
for x = 1:cam.dim(1)
    for y = 1:cam.dim(2)
        if image(y,x,4) == Inf
            % Nothing is captured on this pixel
            continue
        end
        % Scaling factor
        w = image(y,x,4)/sqrt(pos_x(x)^2+pos_y(y)^2+f^2);
        rep(x,y,1) = pos_x(x)*w;
        rep(x,y,2) = pos_y(y)*w;
        rep(x,y,3) = f*w;
        rep(x,y,4) = image(y,x,1);
        rep(x,y,5) = image(y,x,2);
        rep(x,y,6) = image(y,x,3);
    end
end
% Flatten the point cloud to list and remove pixels with no capture.
rep = reshape(rep,[],6);
rep = rep(rep(:,4)<Inf,:);
end

%% Function for converting RGB-D image to Cartesian depth map
function rep = rgbd_to_cartesian(image,cam,f)
rep = Inf(cam.dim(1),cam.dim(2));
% Calculate the coordinates of pixel positions.
pos_x = ((1-cam.dim(1)/2):(cam.dim(1)/2))/cam.res(1);
pos_y = ((1-cam.dim(2)/2):(cam.dim(2)/2))/cam.res(2);
for x = 1:cam.dim(1)
    for y = 1:cam.dim(2)
        if image(y,x,4) == Inf
            % Nothing is captured on this pixel
            continue
        end
        % Scaling factor
        w = image(y,x,4)/sqrt(pos_x(x)^2+pos_y(y)^2+f^2);
        rep(y,x) = f*w;
    end
end
end

%% Function for converting RGB-D image to spherical depth map
function rep = rgbd_to_spherical(image)
rep = image(:,:,4);
end
