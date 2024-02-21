path = {'G-Bulloides', 'G-Ruber', 'G-Sacculifer', 'N-Dutertrei', 'N-Incompta', 'N-Pachyderma', 'Others'};
outF = 'medianIMG';

% Create output folder
if ~exist(outF, 'dir')
    mkdir(outF);
end

% Start of main loop, goes through all folders of the dataset
for K = 1 : length(path)

    %create datastore of the selected folder
    imB = imageDatastore(path{K}, ...
                         'IncludeSubfolders', true, ...
                         'LabelSource','foldernames');
    
    %create new folder in the selected output folder that has the same name of the input folder                  
    mkdir(outF,path{K}); 
    
    %go through each image in the dataStore
    I = 1;
    while I < length(imB.Labels)                

        [imgR, imgC] = size(readimage(imB,I));
        px = zeros(imgR,imgC,16);
        %this loop goes through 16 images at a time and stores the value of
        %each pixel in a 3D matrix 
        for J = 1 : 16
            img = readimage(imB,I);
            for R = 1 : imgR
               for C = 1 : imgC
                   %disp("J=" + J + " R=" + R + " C=" + C)
                   px(R,C,J) = img(R,C);
               end
            end
            I = I + 1;
        end

        img90 = zeros(imgR,imgC);
        img50 = zeros(imgR,imgC);
        img10 = zeros(imgR,imgC);
        
        %this loop processes 10th, 50th and 90th percentile of each group
        %of 16 pixel gathered in the previous step
        for R = 1 : imgR
            for C = 1 : imgC
                pix = px(R,C,:);
                img10(R,C) = median_high(pix(11:15));
                img50(R,C) = median_high(pix(5:10));
                img90(R,C) = median_high(pix(1:4));
            end
        end
        %since we still have greyscale images we need to use uint8 format
        %for pixels values
        img90 = uint8(img90);
        img50 = uint8(img50);
        img10 = uint8(img10);
        
        %every matrix obtained this way is used as a channel in the RGB image
        %and is than saved in the new folder
        imgO = cat(3,img10,img50,img90);
        nome = strcat(outF,'/',path{K},'/',char(imB.Labels(I-1)),'.png');
        imwrite(imgO,nome);
    end
end   


function result = median_high(data)
    midIndex = ceil(numel(data) / 2);
    result = data(midIndex);
end