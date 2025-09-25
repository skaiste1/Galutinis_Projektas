/*
 * world_builder C++ supervisor controller.
 */
#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <string>
#include <math.h>

#include <QFile>
#include <QXmlSimpleReader>
#include <QXmlInputSource>

#include "cblabhandler.h"

// Define the maximum duration of the simulation in seconds
#define MAX_TIME_SECONDS 200.0
#define M_PI 3.14159265358979323846

struct cell_t
{
    int x, y;
} controlCellPath[1024], newCell;

int nCellPath = 0;
int nextPathInd = 0;
int scoreControl = 0;


// Get the e-puck node by DEF name (e.g., "EPUCK")
webots::Node *epuck_node;

#define PATHCUBESIZE (0.15)

struct cell_t getRobotCell()
{
    struct cell_t cell;
    if (epuck_node)
    {
        const double *position = epuck_node->getPosition();
        // std::cout << "e-puck position: x=" << position[0] << " y=" << position[1] << " z=" << position[2] << std::endl;
        cell.x = static_cast<int>(position[0] / PATHCUBESIZE);
        cell.y = static_cast<int>(position[1] / PATHCUBESIZE);
    }
    return cell;
}

void determine_lab_map_centered_on_robot_initial_pos(cbLab *lab)
{
    int cells_width = int(lab->Width() / PATHCUBESIZE + 0.5);
    int cells_height = int(lab->Height() / PATHCUBESIZE + 0.5);
    int lmap_width = cells_width * 4 - 1;
    int lmap_height = cells_height * 4 - 1;
    char lmap[lmap_height][lmap_width];

    memset(lmap, ' ', sizeof(lmap));

    // debug

    struct cell_t initCell = getRobotCell();
    ;

    // find vertical walls
    for (int cy = 0; cy < cells_height; cy++)
    {
        for (int cx = 0; cx < cells_width; cx++)
        {
            if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                                cbPoint((cx + 1.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE)))
            {
                // fprintf(stderr,"not reachable %d %d -> %d %d, lmap %d %d\n", cy, cx, cy, cx+1,
                //          (cy-initCell.y)*2+lmap_height/2,(cx-initCell.x)*2+1+lmap_width/2);
                lmap[(cy - initCell.y) * 2 + lmap_height / 2][(cx - initCell.x) * 2 + 1 + lmap_width / 2] = '|';
            }
        }
    }
    // vertical left wall
    for (int cy = 0; cy < cells_height; cy++)
    {
        int cx = 0;
        if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                            cbPoint((cx - 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE)))
        {
            lmap[(cy - initCell.y) * 2 + lmap_height / 2][(cx - initCell.x) * 2 - 1 + lmap_width / 2] = '|';
        }
    }

    // find horizontal walls
    for (int cy = 0; cy < cells_height; cy++)
    {
        for (int cx = 0; cx < lmap_width; cx++)
        {
            if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                                cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 1.5) * PATHCUBESIZE)))
            {
                lmap[(cy - initCell.y) * 2 + 1 + lmap_height / 2][(cx - initCell.x) * 2 + lmap_width / 2] = '-';
            }
        }
    }
    // horizontal lower wall
    int cy = 0;
    for (int cx = 0; cx < lmap_width; cx++)
    {
        if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                            cbPoint((cx + 0.5) * PATHCUBESIZE, (cy - 0.5) * PATHCUBESIZE)))
        {
            lmap[(cy - initCell.y) * 2 - 1 + lmap_height / 2][(cx - initCell.x) * 2 + lmap_width / 2] = '-';
        }
    }

    // mark initial pos as known
    lmap[lmap_height / 2][lmap_width / 2] = 'X';

    // mark reachable positions
    int changes = 1;
    while (changes)
    {
        changes = 0;
        for (int ly = 1; ly < lmap_height - 1; ly++)
        {
            for (int lx = 1; lx < lmap_width; lx++)
            {
                if (lx % 2 == 0 && ly % 2 == 0)
                    continue;
                if (lmap[ly][lx] == ' ' &&
                    (lmap[ly][lx + 1] == 'X' || lmap[ly][lx - 1] == 'X' || lmap[ly + 1][lx] == 'X' || lmap[ly - 1][lx] == 'X'))
                {
                    changes = 1;
                    lmap[ly][lx] = 'X';
                }
            }
        }
    }

    // unmark unseen walls
    for (int ly = 1; ly < lmap_height - 1; ly++)
    {
        for (int lx = 1; lx < lmap_width - 1; lx++)
        {
            if (lmap[ly][lx] == '-' && lmap[ly - 1][lx] != 'X' && lmap[ly + 1][lx] != 'X')
            {
                lmap[ly][lx] = ' ';
            }
            if (lmap[ly][lx] == '|' && lmap[ly][lx - 1] != 'X' && lmap[ly][lx + 1] != 'X')
            {
                lmap[ly][lx] = ' ';
            }
        }
    }

    int ly = 0;
    for (int lx = 0; lx < lmap_width; lx++)
    {
        if (lmap[ly][lx] == '-' && lmap[ly + 1][lx] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }
    ly = lmap_height - 1;
    for (int lx = 0; lx < lmap_width; lx++)
    {
        if (lmap[ly][lx] == '-' && lmap[ly - 1][lx] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }

    int lx = 0;
    for (int ly = 0; ly < lmap_height; ly++)
    {
        if (lmap[ly][lx] == '|' && lmap[ly][lx + 1] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }

    lx = lmap_width - 1;
    for (int ly = 0; ly < lmap_height; ly++)
    {
        if (lmap[ly][lx] == '|' && lmap[ly][lx - 1] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }

    // mark initial pos as I
    lmap[lmap_height / 2][lmap_width / 2] = 'I';

    FILE *fp = fopen("mapping.out", "w");
    if (fp == NULL)
    {
        fprintf(stderr, "Could not create mapping file\n");
    }
    else
    {
        for (int ly = lmap_height - 1; ly >= 0; ly--)
        {
            for (int lx = 0; lx < lmap_width; lx++)
            {
                fprintf(fp, "%c", lmap[ly][lx]);
            }
            fprintf(fp, "\n");
        }
        fclose(fp);
    }
}

int main(int argc, char **argv)
{
    // ---
    // 1. INITIALIZATION
    // ---
    webots::Supervisor *supervisor = new webots::Supervisor();
    int timeStep = (int)supervisor->getBasicTimeStep();

    // random offsets to avoid GPS informing on exact position in maze
    int offsetX = 0;
    int offsetY = 0;

    srand(time(0));
    offsetX = rand() % 21;
    offsetY = rand() % 21;

    // ---
    // 2. GET SCENE TREE NODES
    // ---

    // Get the root node of the scene tree
    webots::Node *root_node = supervisor->getRoot();
    // Get the 'children' field of the root node, where we'll add new objects.
    webots::Field *children_field = root_node->getField("children");

    // ---
    // 3. DEFINE AND CREATE NEW OBJECTS
    // ---

    char lab_filename[1024 * 8] = "C2-lab.xml";
    QXmlInputSource *source;

    QFile srcFile(lab_filename);

    if (!srcFile.exists())
    {
        std::cerr << "Could not open " << lab_filename << "\n";
        exit(0);
    }
    if ((source = new QXmlInputSource(&srcFile)) == 0)
    {
        std::cerr << "Fail sourcing lab file\n";
        exit(0);
    }

    cbLabHandler *labHandler = new cbLabHandler;
    labHandler->setChildrenField(children_field);
    labHandler->setOffsets(offsetX, offsetY);

    QXmlSimpleReader xmlParser;
    xmlParser.setContentHandler(labHandler);

    xmlParser.setErrorHandler(labHandler);

    if (!xmlParser.parse(source))
    {
        std::cerr << "Error parsing lab file\n";
        exit(0);
    }

    epuck_node = supervisor->getFromDef("EPUCK");
    if (epuck_node == NULL)
    {
        std::cerr << "No DEF EPUCK node found in the current world file\n";
        exit(0);
    }
    
    webots::Field *translationField = epuck_node->getField("translation");
    if (translationField) {
        // Define the new position (e.g., move to X=1.0, Y=2.0)
        double newTranslation[3] = {labHandler->getLab()->Target(0)->Center().x + offsetX * PATHCUBESIZE, 
                                    labHandler->getLab()->Target(0)->Center().y + offsetY * PATHCUBESIZE, 
                                    0.0}; 
        translationField->setSFVec3f(newTranslation);
        //std::cout << "E-puck repositioned." << std::endl;
    }

    determine_lab_map_centered_on_robot_initial_pos(labHandler->getLab());

    // ---
    // 4. MAIN LOOP (Optional)
    // ---

    // The main work is done, but the controller must keep running
    // for the simulation to continue.
    while (supervisor->step(timeStep) != -1)
    {
        // Get the current simulation time.
        double currentTime = supervisor->getTime();

        std::string scoreText;

        // Check if the simulation time has exceeded the maximum duration.
        if (currentTime >= MAX_TIME_SECONDS)
        {
            // The argument 0 indicates a successful exit.
            if (epuck_node) {
                epuck_node->remove();
                epuck_node = NULL;
            }
            scoreText = "GAME OVER";
        }
        else {
            scoreText = "";
        }

        // Display the label
        supervisor->setLabel(0, scoreText, 0.6, 0.01, 0.1, 0xFF0000, 0.0, "Arial");


    }

    delete supervisor;
    return 0;
}
