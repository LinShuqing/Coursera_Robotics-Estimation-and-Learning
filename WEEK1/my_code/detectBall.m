% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = cell2mat(struct2cell(load('rgb_mu.mat')));
sig = cell2mat(struct2cell(load('rgb_sigma.mat')));
thre = 0.00002;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
[height,width,~] = size(I);
[~,dim] = size(mu);
%  mask_pob=zeros(height,width);
prob_img = zeros(height,width);
for ii=1:height
    for jj=1:width
        prob_img(ii,jj)=mvnpdf(double([I(ii,jj,1),I(ii,jj,2),I(ii,jj,3)]),mu,sig);
    end
end
prob_mask = prob_img>thre;
figure,surf(prob_img);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
segI=false(height,width);
CC=bwconncomp(prob_mask);
numPixels = cellfun(@numel,CC.PixelIdxList);
[~,idx] = max(numPixels);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI(CC.PixelIdxList{idx}) = true; 
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
