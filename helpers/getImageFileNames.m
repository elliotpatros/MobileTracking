function imageFileNames = getImageFileNames(directory, nImages, extension)

imageFileNames = cell(1, nImages);

for n = 1:nImages
    imageFileNames(1, n) = {[directory, '-', num2str(n), '.', extension]};
end

end

