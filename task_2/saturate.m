function output = saturate(input, lower_limit, upper_limit)

if (input < lower_limit)
    input = lower_limit;
elseif (input > upper_limit)
    input = upper_limit;
end

output = input;

end