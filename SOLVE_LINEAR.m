function x = SOLVE_LINEAR(A,b)

    SIZE =length(b);

    L = chol(A);

    R = L';
    x = zeros(SIZE,1);
    z = zeros(SIZE,1);
    
        for i = 1: SIZE

            num = b(i);
            for j = 1 : i - 1
              num = num - R(i,j)*z(j);
            end

            z(i) = num/R(i,i);

        end


        for i= 1:SIZE

            true_i = SIZE - i + 1;
            num = z(true_i);

            for j = 1 : i - 1
                true_j = true_i+j;
                num = num - L(true_i,true_j)*x(true_j);
            end

            x(true_i) = num/L(true_i,true_i);
        end



end
