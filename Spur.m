im=imread('C:\Users\Dhaval\Desktop\a.jpg');
imHSV = rgb2hsv(im); % Get the saturation channel for segmentation&
saturation = imHSV(:, :, 2); % saturating the image%
t = graythresh(saturation);  % Threshold the image
imCoin = (saturation > t);
imgear=imCoin;
gearBW = bwareaopen(imgear, 250);
%%imshow(gearBW);
gearBW = imfill(gearBW, 'holes');
imshow(gearBW)
rp = regionprops(double(gearBW), 'all');
xy = rp.ConvexHull;
[r,c] = size(gearBW);
mask = poly2mask(xy(:,1), xy(:,2), r, c);
teeth=xor(mask,gearBW);
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,... % this will set a port to generate a matrix with all filtered out areas of connected points 
      'MinimumBlobArea', 100, 'ExcludeBorderBlobs', true); % this will set a port to generate a matrix that will let us to choose areas greater than k value defined above
    
[areas] = step(blobAnalysis, teeth); % this will detect areas of connected point and will output all the parameters that are required
[~, idx] = sort(areas, 'Descend');
n = size(areas,1);
im1=0;im2=0;
for i=1:rp.MajorAxisLength/2
    maskEroded = imerode(mask, ones(i));
    im1=and(maskEroded,gearBW);
    im2=xor(maskEroded,im1);
    im3=any(im2);
    for j = 1:size(im3,2);
        if im3(1,j)==1
            flag=1;
            k=i;
            break;
        else
            flag=0;
        end
    end
    if flag==0
        inner=maskEroded;
        break;
    else
        continue
    end
end
RP=regionprops(inner,'all');
E=RP.Extrema;
E1=[E(1,1),E(1,2);E(5,1),E(5,2)];
E2=[E(3,1),E(3,2);E(7,1),E(7,2)];
e=rp.Extrema;
e1=[e(1,1),e(1,2);e(5,1),e(5,2)];
e2=[e(3,1),e(3,2);e(7,1),e(7,2)];
Eroded = imerode(mask, ones(ceil(k\2)));
teeth=xor(Eroded,gearBW);
[imagePoints, boardSize] = detectCheckerboardPoints('S:\camera_calib\Image6.png');

squareSize = 89.5; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%[imagePoints, boardSize] = detectCheckerboardPoints(ss);

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
imagePoints1=E1;
imagePoints2=E2;
imagePoints3=e1;
imagePoints4=e2;
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
worldPoints2 = pointsToWorld(cameraParams, R, t, imagePoints2);
worldPoints3 = pointsToWorld(cameraParams, R, t, imagePoints3);
worldPoints4 = pointsToWorld(cameraParams, R, t, imagePoints4);
D1=hypot(worldPoints1(1,1),worldPoints1(1,2),worldPoints1(2,1),worldPoints1(2,2))/10; 
D2=hypot(worldPoints2(1,1),worldPoints2(1,2),worldPoints2(2,1),worldPoints2(2,2))/10; 
d1=hypot(worldPoints3(1,1),worldPoints3(1,2),worldPoints3(2,1),worldPoints3(2,2))/10; 
d2=hypot(worldPoints4(1,1),worldPoints4(1,2),worldPoints4(2,1),worldPoints4(2,2))/10; 
D=(D1+D2)/2;
d=(d1+d2)/2;