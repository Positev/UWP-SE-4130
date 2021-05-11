
const int SIDE_LENGTH = 100;
const float DEGREE_MUTIPLIER = 0.5F; //Hard surface. It depends on surface of groud.

class PathData
{

public:
    int shape_type; //0:circle, 3:triangle, 4: square, 5:pentagon
    int path_state;
    int *movement_type;   //i=0: forward, i=1:spin.
    int *movement_target; //j is movement target.

    PathData();
    PathData(int);
};

//Default constructor
PathData::PathData()
{
    int type = 3;
    shape_type = type;
    path_state = type + 2;
    movement_type = new int[5];
    movement_target = new int[5];
    if (type == 3)
    {
        int t1[5] = {0, 1, 0, 1, 0};
        for (int i = 0; i < 5; i++)
            movement_type[i] = t1[i];
        int t2[5] = {60, 140, 240, 290, 390};
        for (int i = 0; i < 5; i++)
            movement_target[i] = t2[i];
    }
    else
    {
        int t1[5] = {0, 1, 0, 1, 0};
        for (int i = 0; i < 5; i++)
            movement_type[i] = t1[i];
        int t2[5] = {60, 140, 240, 290, 390};
        for (int i = 0; i < 5; i++)
            movement_target[i] = t2[i];
    }
}

PathData::PathData(int type)
{

    shape_type = type;
    path_state = 2 * type - 1;
    movement_type = new int[2 * type - 1];
    movement_target = new int[2 * type - 1];

    int rotation_angle = 0;
    if (type == 3)
    {
        //int t2[2 * type - 1] = {100, 160, 260, 320, 420};
        rotation_angle = 120;
    }
    else if (type == 4)
    {
        rotation_angle = 90;
    }
    else if (type == 5)
    {
        rotation_angle = 82;
    }
    else if (type == 16)
    {
        rotation_angle = 23;
    }

    for (int i = 0; i < 2 * type - 1; i++)
    {
        if (i == 0)
        {
            movement_target[i] = SIDE_LENGTH;
            movement_type[i] = 0;
        }
        else if (i % 2 == 1)
        {
            movement_target[i] = movement_target[i - 1] + DEGREE_MUTIPLIER * rotation_angle;
            movement_type[i] = 1;
        }
        else if (i % 2 == 0)
        {
            movement_target[i] = movement_target[i - 1] + SIDE_LENGTH;
            movement_type[i] = 0;
        }
    }
}
