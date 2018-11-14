function [y,angle] =  calc_center(x,im)
format shortg
start = clock;
y = [];
angle = NaN;
im_gr=rgb2gray(im);
im_bw=im2bw(im_gr, 0.1);

im_bw_1 = ~bwareaopen(~im_bw, 4500);
im_bw_2 = bwareaopen(im_bw_1, 1000);

RegProp = regionprops(~im_bw_2);
data_pos = [cat(1,RegProp.Centroid) cat(1,RegProp.Area) cat(1,RegProp.BoundingBox)];

if size(data_pos,1)
    data_pos = data_pos(data_pos(:,3)<16000,:);
end
if size(data_pos,1)
    data_pos = data_pos(data_pos(:,3)>7000,:);
end
b = [-5,-5,10,10];
if size(data_pos,1)
    data_pos(:,4:7) = data_pos(:,4:7) + (ones(size(data_pos(:,4:7)))*diag(b));

    for i=1:size(data_pos,1)
        crop_img=imcrop(im_bw_1,data_pos(i,4:7));
        crop_imgbw = bwareaopen(crop_img, 10);
        a1 = regionprops(crop_imgbw);
        if size(a1,1)==x
            y = data_pos(i,1:2);
            Acell = [cat(1,a1.Centroid) cat(1,a1.Area)]  ;       
            Acell = sortrows(Acell, -3);
            angle = rad2deg(atan2((Acell(3,1)-Acell(2,1)),(Acell(3,2)-Acell(2,2))));
            break;
        end
    end
end
endt = clock-start ;
end

