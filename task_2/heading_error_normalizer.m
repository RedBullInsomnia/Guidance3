function[heading_error] = heading_error_normalizer(heading_error)

heading_error = rem(heading_error, (2 * pi));

if (heading_error > pi)
    heading_error = heading_error - 2 * pi;
elseif (heading_error < -pi)
    heading_error = heading_error + 2 * pi;
end