//Couleur des zones
enum Couleur{JAUNE, BLEU};
enum Type {JARDINIERE, SERRE};

typedef struct _zone{
    //Numéro de la zone
    int zone_id; 

    //Couleur de la zone
    Couleur couleur;

    //Type de la zone
    Type type;

    //Points de la diagonale
    float x_1;
    float y_1;
    float x_2;
    float y_2;

    //Le centre de la zone
    float x_center;
    float y_center;
} Zone;

//6 zones par couleur
const Zone zones_bleues[6] = {
    {1, BLEU, JARDINIERE, -775, -1275, -550, -1050, -775, -1275},
    {2, BLEU, JARDINIERE, -550, -925, -775, -1275, -550, -925},
    {3, BLEU, JARDINIERE, -925, -737.5, -775, -1275, -925, -737.5},
    {4, BLEU, SERRE, 775, -1275, -550, -1500, 1000, -1050},
    {5, BLEU, SERRE, 0, 1275, -225, 150, 225, 1500},
    {6, BLEU, SERRE, 0, 1275, -225, 150, 225, 1500}
};

const Zone zones_jaunes[6] = {
    {1, JAUNE, JARDINIERE, 775, 1275, 550, 1050, 775, 1275},
    {2, JAUNE, JARDINIERE, 550, 925, 775, 1275, 550, 925},
    {3, JAUNE, JARDINIERE, 925, 737.5, 775, 1275, 925, 737.5},
    {4, JAUNE, SERRE, -775, 1275, 550, 1500, -1000, 1050},
    {5, JAUNE, SERRE, 0, -1275, 225, -150, -225, -1500},
    {6, JAUNE, SERRE, 0, -1275, 225, -150, -225, -1500}
};

