#ifndef MAG3110_H
#define MAG3110_H


class MAG3110
{
    public:
        MAG3110();
        virtual ~MAG3110();
        int read_x();
        int read_y();
        int read_z();
        void config();

    protected:

    private:
        

};

#endif // MAG3110_H
