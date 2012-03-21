%com = input('Enter COM port: ');
s = serial('COM5');
fopen(s);
while(1)
    out = fscanf(s, '%d');
    out
end
fclose(s);