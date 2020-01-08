function[D,d,n]=GearDimensions(im,cameraParams,im1,squaresize,K) 
% Inputs    im:              Gear Image captured for measurements
%           cameraParams:    Calibrated camera Parameters
%           im1:             One of the sample images taken while calibration that
%                            closely resembles the position of checkerboard with the gear in im (In this case Centre)
%           squaresize:      size of checkerboard square in mm
%           k:               minimum blob area to be considered for teeth calculation,
%                            depends on size of teeths in captured image.
%
%Output     D:               Outer Diameter(approx)
%           d:               Inner Diameter(approx)
%           n:               Number of teeths

%% Segmenting the image
imHSV = rgb2hsv(im); 
saturation = imHSV(:, :, 2); 
t = graythresh(saturation);  
imCoin = (saturation > t);
imgear=imCoin;
gearBW = bwareaopen(imgear, 200);
gearBW = imfill(gearBW, 'holes');
%% Making ConvexHull for outer boundary of ROI
rp = regionprops(double(gearBW), 'all');
xy = rp.ConvexHull;
[r,c] = size(gearBW);
mask = poly2mask(xy(:,1), xy(:,2), r, c);

%% Finding ID by shrinking outer convexhull
for i=1:rp.MajorAxisLength/2
    maskEroded = imerode(mask, ones(i));
    im1=and(maskEroded,gearBW);
    im2=xor(maskEroded,im1);
    im3=any(im2);
    for j = 1:size(im3,2)
        if im3(1,j)==1
            flag=1;
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
% E's and e's are extreme points of ConvexHull representing Id and Od
%% Extracting teeths and counting number of teeths
Eroded = imerode(mask, ones(ceil(k\2)));
teeth=xor(Eroded,gearBW);
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,... 
      'MinimumBlobArea', K , 'ExcludeBorderBlobs', true);
[areas] = step(blobAnalysis, teeth); 
n = size(areas,1);
%% Camera Calibration & Id Od calculation
[imagePoints, boardSize] = detectCheckerboardPoints(im1);
worldPoints = generateCheckerboardPoints(boardSize, squaresize);
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
imagePoints1 = E1;
imagePoints2 = E2;
imagePoints3 = e1;
imagePoints4 = e2;
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
worldPoints2 = pointsToWorld(cameraParams, R, t, imagePoints2);
worldPoints3 = pointsToWorld(cameraParams, R, t, imagePoints3);
worldPoints4 = pointsToWorld(cameraParams, R, t, imagePoints4);
d1=hypot(worldPoints1(1,1),worldPoints1(1,2),worldPoints1(2,1),worldPoints1(2,2))/10; 
d2=hypot(worldPoints2(1,1),worldPoints2(1,2),worldPoints2(2,1),worldPoints2(2,2))/10; 
D1=hypot(worldPoints3(1,1),worldPoints3(1,2),worldPoints3(2,1),worldPoints3(2,2))/10; 
D2=hypot(worldPoints4(1,1),worldPoints4(1,2),worldPoints4(2,1),worldPoints4(2,2))/10; 
D=(D1+D2)/2;
d=(d1+d2)/2;
