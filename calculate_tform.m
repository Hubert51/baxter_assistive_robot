function [ tform, theta ] = calculate_tform( queryImg, trainImg )
    queryImg_copy = queryImg;
    for i=0:2
        for j=0:2
            for k=1:100
                if i == 0 || j == 0

                else 
                    queryImg = imcrop(queryImg_copy, [ round((i-1)*w/3),  round((j-1)*h/3), round((i+1)*w/3),  round((j+1)*h/3) ]);
                end

                try
                    [tform, theta] = featureMatcher( queryImg, trainImg );
                    k = 100;
                    i = 2;
                    j = 2;
                    return 
                catch
                    continue
                end

            end
        end
    end
end