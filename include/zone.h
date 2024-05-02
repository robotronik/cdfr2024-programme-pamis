//Type de zones
enum Type {JARDINIERE, SERRE};

typedef struct _zone{
    //Numéro de la zone
    int zone_id; 

    //Type de la zone
    Type type;

    //Le centre de la zone
    float x_center;
    float y_center;
    
    //Points de la diagonale
    float x_1;
    float y_1;
    float x_2;
    float y_2;

} Zone;