#include <iostream>
#include <queue>
#include <climits>
using namespace std;



const int MAXN = 1000;

// Define the node class
class Node {
public:
    int x, y, weight;
    char symbol;

    Node()
    {
        weight = 1;
        symbol = '*';
    }

    void setyco(int y)
    {
        this->y = y;
    }

    int getweight()
    {
        return weight;
    }
    void setxco(int x)
    {
        this->x = x;
    }


    Node(int x_, int y_, int weight_, char s) : x(x_), y(y_), weight(weight_), symbol(s) {}
};

// Define the Graph class
class Graph {
public:
    int V;
    vector<pair<int, int>>* adj;

    Graph(int V_) {
        V = V_;
        adj = new vector<pair<int, int>>[V];
    }
    Graph()
    {

    }

    void addEdge(int u, int v, int w) {
        adj[u].push_back({ v, w });
        adj[v].push_back({ u, w });
    }
};

// Define the Dijkstra class
class Dijkstra {
public:
    vector<int> sumWeights;
    vector<bool> visited;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    Graph graph;

    Dijkstra(Graph graph_, int start, vector<int>& parent)
    {
        graph = graph_;
        sumWeights.assign(graph.V, INT_MAX);
        visited.assign(graph.V, false);
        parent.assign(graph.V, -1);
        sumWeights[start] = 0;
        parent[start] = start;
        pq.push({ 0, start });

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (visited[u]) {
                continue;
            }

            visited[u] = true;

            for (auto edge : graph.adj[u]) {
                int v = edge.first;
                int w = edge.second;
                if (sumWeights[u] + w < sumWeights[v]) {
                    sumWeights[v] = sumWeights[u] + w;
                    parent[v] = u;
                    pq.push({ sumWeights[v], v });
                }
            }
        }
    }

};




vector<int> getShortestPathandfindfitness(int start, int end, const vector<int>& parent, Node** arr, int& shortestpathcost)
{
    vector<int> path;
    int start_ind1 = -1;
    int start_ind2 = -1;

    int destination_ind2 = -1;

    for (int v = end; v != start; v = parent[v]) {
        path.push_back(v);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    for (int i = 0; i < path.size(); i++)
    {
        int ind1 = path[i] / 40;
        int ind2 = path[i] % 40;
        if (arr[ind1][ind2].symbol == 'L')
        {
            shortestpathcost = shortestpathcost + 10;   //penlaities given according to Assignment 1
        }
        else if (arr[ind1][ind2].symbol == 'H')
        {
            shortestpathcost = shortestpathcost + 20;
        }

    }
    return path;
}

void print2darray(Node** arr)
{
    for (int i = 0;i < 40;i++)
    {
        for (int j = 0;j < 40;j++)
        {
            cout << "x: " << i << " y: " << j << "  weight: " << arr[i][j].weight << endl;
        }
    }
}

void print2dgrid(Node** arr)
{
    for (int i = 0;i < 40;i++)
    {
        for (int j = 0;j < 40;j++)
        {
            cout << arr[i][j].symbol;
        }
        cout << endl;
    }
}




void printing(Node** arr)
{
    for (int i = 0; i < 20; i++)
    {
        cout << "in chromosome " << i + 1 << endl;
        for (int j = 0; j < 5; j++)
        {
            cout << "in gene " << j + 1 << endl;
            cout << "xco: " << arr[i][j].x << "  yco: " << arr[i][j].y << endl;

        }
        cout << endl;
    }
}



bool diversityinfitness(float* arr)
{
    bool flag = true;
    for (int i = 0;i < 19;i++)
    {
        if (arr[i] != arr[i + 1])
            flag = false;
    }
    return flag;
}

int main() {
    // Read input
    int n = 40;
    int m = 40;

    // Create the node array and fill it with input values
    Node** arr = new Node * [n];
    for (int i = 0; i < n; i++) {
        arr[i] = new Node[m];
        for (int j = 0; j < m; j++) {
            int weight, x, y;
            arr[i][j] = Node(i, j, 1, '*');
        }
    }


    srand(time(0));
    int start_ind1 = rand() % 40;
    int start_ind2 = rand() % 40;
    int destination_ind1 = rand() % 40;
    int destination_ind2 = rand() % 40;
    arr[start_ind1][start_ind2].symbol = 'S';


    arr[start_ind1][start_ind2].setxco(start_ind1);
    arr[start_ind1][start_ind2].setyco(start_ind2);
    arr[destination_ind1][destination_ind2].setxco(destination_ind1);
    arr[destination_ind1][destination_ind2].setyco(destination_ind2);


    arr[destination_ind1][destination_ind2].symbol = 'D';


    int nooflighttraffic = rand() % 40;
    int heavytraffic = rand() % 40;

    for (int i = 0;i < nooflighttraffic;i++)
    {
        int randomind1 = rand() % 40;
        int randomind2 = rand() % 40;
        if (arr[randomind1][randomind2].symbol != 'S' && arr[randomind1][randomind2].symbol != 'D')
            arr[randomind1][randomind2].symbol = 'L';
        else
            i--;

    }

    for (int i = 0;i < heavytraffic;i++)
    {

        int randomind1 = rand() % 40;
        int randomind2 = rand() % 40;
        if (arr[randomind1][randomind2].symbol != 'S' && arr[randomind1][randomind2].symbol != 'D')
            arr[randomind1][randomind2].symbol = 'H';
        else
            i--;

    }

    int costs[20] = { -1 };
    Node** population = new Node * [20];
    for (int i = 0;i < 20;i++)
    {
        population[i] = new Node[5];
        int cost = 0;
        int x1;
        int y1;

        for (int j = 0;j < 5;j++)   //making a chromosome
        {
            x1 = 0 + rand() % (39 - 0 + 1);
            y1 = 0 + rand() % (39 - 0 + 1);



            if (j == 0)
            {
                population[i][j] = arr[start_ind1][start_ind2];

            }
            else if (j == 4)
            {
                population[i][j] = arr[destination_ind1][destination_ind2];

            }
            else
            {
                if (arr[x1][y1].symbol == 'o')
                {
                    //duplication
                    j--;
                }
                else
                {
                    arr[x1][y1].symbol = 'o';
                    population[i][j].symbol = 'o';
                    population[i][j].x = x1;
                    population[i][j].y = y1;
                    population[i][j].weight = 1;
                }


            }
            cout << population[i][j].symbol << " ";
            cout << population[i][j].x << " ";
            cout << population[i][j].y << endl;

        }



        print2dgrid(arr);

        //we nedd to find distances between each node in the chromosome

        //print2d(arr);


    }
    cout << endl << endl;


    print2dgrid(arr);
    cout << endl << endl << endl;
    // Create a graph from the node array
    Graph graph(n * m);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            int u = i * m + j;

            if (j > 0) {
                int v = i * m + (j - 1);
                int w = arr[i][j - 1].weight;
                graph.addEdge(u, v, w);
            }

            if (j < m - 1) {
                int v = i * m + (j + 1);
                int w = 1; // assuming each node has a cost of 1
                graph.addEdge(u, v, w);
            }

            if (i > 0) {
                int v = (i - 1) * m + j;
                int w = 1; // assuming each node has a cost of 1
                graph.addEdge(u, v, w);
            }

            if (i < n - 1) {
                int v = (i + 1) * m + j;
                int w = 1; // assuming each node has a cost of 1
                graph.addEdge(u, v, w);
            }
        }
    }


    float costofchromosomes[20];  //fitness of ecah chromosome 
    float fitnessofchromosomes[20];
    int noofite = 0;   //To limit the number of iterations if best fitness is not being reached 

    while (noofite < 150)  //150
    {
        noofite++;
        float roulettewheel[20];
        float sumofallfitnessvalues = 0.0f;
        float temp = 0.0f;
        int backupcosts[22];

        for (int i = 0;i < 20;i++)
        {
            int shortestpath_perchromosome = INT_MAX;
            for (int j = 0;j < 4;j++)
            {

                int sidx = (population[i][j].x * m) + population[i][j].y;
                int didx = (population[i][j + 1].x * m) + population[i][j + 1].y;
                vector<int>parent;

                // Run Dijkstra's algorithm to find the shortest path
                Dijkstra dijkstra(graph, sidx, parent);
                int shortestPath = dijkstra.sumWeights[didx];


                vector<int> path = getShortestPathandfindfitness(sidx, didx, parent, arr, shortestPath);

                // Print the shortest path cose
                if (shortestPath == INT_MAX)
                {
                    shortestpath_perchromosome = INT_MAX;
                    break;
                }
                if (j == 0)
                    shortestpath_perchromosome = shortestPath;
                else
                    shortestpath_perchromosome = shortestpath_perchromosome + shortestPath;

                cout << "shortest path from one point to another: " << shortestPath << endl;


            }
            cout << "total cost of shortest path of chromosome " << i + 1 << " is: " << shortestpath_perchromosome << endl;
            costofchromosomes[i] = shortestpath_perchromosome;
            backupcosts[i] = costofchromosomes[i];  //back up conatins costs
        }


        //now we have a list of all the costs of chromosomes
        //in this list,  more fitness means smaller the value
        //we have to change it

        int max = costofchromosomes[0];
        for (int i = 1; i < 20;i++)
        {
            if (costofchromosomes[i] > max && costofchromosomes[i] != INT_MAX)
                max = costofchromosomes[i];
        }
        max = max + 1;
        //we added 1 because we do not want the least fit  to have  zero fitness after manipulation
        //For eaxmple, if the cost of best fit is 3 and the cost of worst fit (not including INT_MAX) is 14 
        //then 14-14 will equal to 0 leading to 0 probability for becoming parent, BUT it should have some probability becaue only chromosomes having
        // INT_MAX as costs will have zero probability
        // thus we make max=14+1=15 instead of 14. Now 15-14=1 (small probability) for worst fit.
        //THIS WILL NOT AFFECT THE PROBABILITIES OF OTHERS 


        for (int i = 0;i < 20;i++)
        {
            if (costofchromosomes[i] == INT_MAX)
                fitnessofchromosomes[i] = 0;
            else
                fitnessofchromosomes[i] = max - costofchromosomes[i];       //now more the fitness, greater the value 
            sumofallfitnessvalues = sumofallfitnessvalues + fitnessofchromosomes[i];
        }

        if (diversityinfitness(fitnessofchromosomes) == true)
            break;


        // we use proportionate selection algorithm
        for (int i = 0;i < 20;i++)
        {
            cout << "fitness value before division: " << fitnessofchromosomes[i] << endl;
            fitnessofchromosomes[i] = (fitnessofchromosomes[i] / sumofallfitnessvalues);
        }

        float number = 0.0f;
        for (int i = 0;i < 20;i++)
        {
            cout << "Realtive fitness value: " << fitnessofchromosomes[i] << endl;
            number = fitnessofchromosomes[i] * 200;
            roulettewheel[i] = temp +number;     //mapping the roulette by mutiplying a factor of 200 Probabilties remain the same 
            //Done to avoid the selection of only 0 and 1 chromosomes as selection probability is VERY small 
            temp = roulettewheel[i];
            cout << "roulette wheel value: " << roulettewheel[i] << endl;
        }


        double  firstparent;
        double  secondparent;
        int fparentind = -1;
        int sparentind = -1;
        //srand(time(NULL));

        do
        {
            double max = roulettewheel[19];  //the upper bound of value 
            cout << roulettewheel[19] << endl;
            double min = roulettewheel[0];
            firstparent = 1 + rand() % (long long(max) - 1 + 1);

            secondparent = 1 + rand() % (long long(max) - 1 + 1);
            cout << "1st parent value selected randomly: " << firstparent << "   " << "2nd parent value selected randomly: " << secondparent << endl;
            for (int i = 1;i < 20;i++)
            {
                if (firstparent <= min)
                    fparentind = 0;

                else if (firstparent <= roulettewheel[i] && firstparent > roulettewheel[i - 1])
                    fparentind = i; //chromosome number

                if (secondparent <= min)
                    sparentind = 0;

                else if (secondparent <= roulettewheel[i] && secondparent > roulettewheel[i - 1])
                    sparentind = i; //chromosome number
            }

        } while (fparentind == sparentind);
        cout << "1st parent ind: " << fparentind << "   " << "2nd parent ind: " << sparentind << endl;


        //We do two step crossover

        Node* newchromosome1 = new Node[5];
        Node* newchromosome2 = new Node[5];
        for (int i = 0;i < 5;i++)
        {

            newchromosome1[i] = Node();
            newchromosome2[i] = Node();

        }



        int cut1 = 1;
        int cut2 = 3;
        for (int i = 0;i < 5;i++)  //for new chromoseom 1
        {
            if (i < cut1)
            {

                newchromosome1[i] = population[fparentind][i];

            }

            else if (i >= cut1 && i < cut2)
            {

                newchromosome1[i] = population[sparentind][i];

            }
            else if (i >= cut2)
            {

                newchromosome1[i] = population[fparentind][i];

            }
        }



        for (int i = 0;i < 5;i++)  //for new chromoseom 2
        {
            if (i < cut1)
            {

                newchromosome2[i] = population[sparentind][i];

            }

            else if (i >= cut1 && i < cut2)
            {


                newchromosome2[i] = population[fparentind][i];

            }
            else if (i >= cut2)
            {

                newchromosome2[i] = population[sparentind][i];

            }

        }


        int num = 20;

        for (int i = 0;i < 2;i++)   //finding costs of new chromosomes 
        {
            int shortestpath_perchromosome = INT_MAX;
            for (int j = 0;j < 4;j++)
            {

                int sidx = (population[i][j].x * m) + population[i][j].y;
                int didx = (population[i][j + 1].x * m) + population[i][j + 1].y;
                vector<int>parent;

                // Run Dijkstra's algorithm to find the shortest path
                Dijkstra dijkstra(graph, sidx, parent);
                int shortestPath = dijkstra.sumWeights[didx];


                vector<int> path = getShortestPathandfindfitness(sidx, didx, parent, arr, shortestPath);

                // Print the shortest path cose
                if (shortestPath == INT_MAX)
                {
                    shortestpath_perchromosome = INT_MAX;
                    break;
                }
                if (j == 0)
                    shortestpath_perchromosome = shortestPath;
                else
                    shortestpath_perchromosome = shortestpath_perchromosome + shortestPath;

                cout << "shortest path from one point to another: " << shortestPath << endl;


            }
            cout << "total cost of shortest path of chromosome " << num << " is: " << shortestpath_perchromosome << endl;
            backupcosts[num] = shortestpath_perchromosome;  //back up conatins costs
            num++;
        }



        //now we find fitnesses to remove two chromosomes from the population
        int maxno = backupcosts[0];
        for (int i = 1; i < 22;i++)
        {
            if (backupcosts[i] > maxno && backupcosts[i] != INT_MAX)
                maxno = backupcosts[i];
        }
        maxno = maxno + 1;

        float newfitnessvalues[22];
        cout << "All fitness values including new chromosome: " << endl;

        for (int i = 0;i < 22;i++)
        {
            if (backupcosts[i] == INT_MAX)
            {
                newfitnessvalues[i] = 0;
            }
            else
            {
                newfitnessvalues[i] = maxno - backupcosts[i];       //now more the fitness, greater the value 
            }
            cout << "Fitness: " << newfitnessvalues[i] << endl;
        }




        //finding minimum fit chromoseomes
        int minfitness = newfitnessvalues[0];
        int secondminfitness = INT_MAX;
        int minfitnessind = 0;
        int secondminfitnessind = -1;
        for (int i = 1; i < 22; i++)
        {
            if (newfitnessvalues[i] < minfitness)
            {
                minfitness = newfitnessvalues[i];
                minfitnessind = i;
            }
        }
        cout << "min fitness: " << minfitness << " at index: " << minfitnessind << endl;

        for (int i = 0; i < 22; i++)
        {
            if (newfitnessvalues[i] < secondminfitness && (newfitnessvalues[i] >= minfitness && i != minfitnessind))   //>= in case od dupliacte values 
            {
                secondminfitness = newfitnessvalues[i];
                secondminfitnessind = i;
            }
        }
        cout << "second min fitness: " << secondminfitness << " at index: " << secondminfitnessind << endl;

        //now we have two most unfit chromosomes


        //we do this to avoid copying the entire population
        // we make changes into the existing one
        // to save space 
        if (minfitnessind == 20 || secondminfitnessind == 20)
        {

            delete[] newchromosome1;
        }

        if (minfitnessind == 21 || secondminfitnessind == 21)
        {


            delete[] newchromosome2;
        }

        if (minfitnessind <= 19 && secondminfitnessind <= 19)   //population size is 20
        {
            for (int i = 0;i < 5;i++)
            {

                population[minfitnessind][i] = newchromosome1[i];
                population[secondminfitnessind][i] = newchromosome2[i];

            }

        }

        if (minfitnessind <= 19 && secondminfitnessind == 20)
        {

            for (int i = 0;i < 5;i++)
            {

                population[minfitnessind][i] = newchromosome2[i];

            }

        }

        else if (minfitnessind <= 19 && secondminfitnessind == 21)
        {

            for (int i = 0;i < 5;i++)
            {

                population[minfitnessind][i] = newchromosome1[i];

            }

        }

        else if (secondminfitnessind <= 19 && minfitnessind == 20)
        {
            for (int i = 0;i < 5;i++)
            {

                population[secondminfitnessind][i] = newchromosome2[i];

            }
        }

        else if (secondminfitnessind <= 19 && minfitnessind == 21)

        {
            for (int i = 0;i < 5;i++)
            {

                population[secondminfitnessind][i] = newchromosome1[i];

            }
        }



        //now we have a new population 
        int limit = 20, j = 0;
        for (int i = 0;i < limit;i++)
        {
            if (i != minfitnessind && i != secondminfitnessind)
            {
                costofchromosomes[j] = backupcosts[i];
                fitnessofchromosomes[j] = newfitnessvalues[i];
                j++;
            }
            else
            {
                limit++;
            }

        }


        cout << "All fitness values (not in order) for our NEW population after removing 2 least fit solutions are: " << endl;
        for (int i = 0;i < 20;i++)
        {
            cout << "Fitness : " << fitnessofchromosomes[i] << endl;
        }

        cout << "The new population is: " << endl;
        printing(population);

        bool flag2 = diversityinfitness(fitnessofchromosomes);
        if (flag2 == true)
        {
            break;
        }

    }


    if (noofite >= 150)  //150
    {
        cout << "ITERATIONS UPPER BOUND REACHED " << endl;
        cout << "All possible solutions with 150 iterations: " << endl;
        printing(population);
        cout << endl << endl;
        float min = costofchromosomes[0];
        for (int i = 1;i < 20;i++)
        {
            if (costofchromosomes[i] < min)
                min = costofchromosomes[i];
        }
        cout << "cost of most optimal solution(s) with 150 iterations: " << min << endl;
        cout << "most optimal solution with 150 iterations: " << endl;
        //we do this to find the path
        for (int i = 0;i < 20;i++)
        {
            vector<int>TotalPath;
            int shortestpath_perchromosome = INT_MAX;
            vector<int>path;
            int aindex = 0;
            int indexarray[3];
            for (int j = 0;j < 4;j++)
            {
                int sidx = (population[i][j].x * m) + population[i][j].y;
                int didx = (population[i][j + 1].x * m) + population[i][j + 1].y;
                vector<int>parent;
                if (j >= 1 && j <= 3)
                {

                    indexarray[aindex] = sidx;
                    aindex++;
                }
                // Run Dijkstra's algorithm to find the shortest path
                Dijkstra dijkstra(graph, sidx, parent);
                int shortestPath = dijkstra.sumWeights[didx];


                path = getShortestPathandfindfitness(sidx, didx, parent, arr, shortestPath);
                if (j == 0)
                    TotalPath = path;
                else
                {
                    TotalPath.insert(TotalPath.end(), path.begin(), path.end());
                }
                // Print the shortest path cose
                if (shortestPath == INT_MAX)
                {
                    shortestpath_perchromosome = INT_MAX;
                    break;
                }
                if (j == 0)
                    shortestpath_perchromosome = shortestPath;
                else
                    shortestpath_perchromosome = shortestpath_perchromosome + shortestPath;


            }
            if (shortestpath_perchromosome == min)
            {
                print2dgrid(arr);
                cout << "the shortest path is : ";
                for (int i = 0; i < TotalPath.size(); i++)
                {

                    int ind1 = TotalPath[i] / 40;
                    int ind2 = TotalPath[i] % 40;
                    if (TotalPath[i] == indexarray[0] || TotalPath[i] == indexarray[1] || TotalPath[i] == indexarray[2])
                        arr[ind1][ind2].symbol = 'O';
                    else
                        arr[ind1][ind2].symbol = '#';

                    arr[start_ind1][start_ind2].symbol = 'S';
                    arr[destination_ind1][destination_ind2].symbol = 'D';
                    cout << "(" << ind1 << "," << ind2 << ")" << "  ";


                }
                arr[start_ind1][start_ind2].symbol = 'S';
                arr[destination_ind1][destination_ind2].symbol = 'D';

                cout << endl;
                print2dgrid(arr);
                cout << endl;
                break;
            }

        }

    }

    else if (diversityinfitness(fitnessofchromosomes) == true)
    {
        cout << "No diversity left " << endl;
        cout << "Maximum POSSIBLE cost for reaching is: " << costofchromosomes[0] << endl;  //as all values are same 
        cout << "All possible solutions are: " << endl;
        printing(population);


        float min = costofchromosomes[0];
        for (int i = 1;i < 20;i++)
        {
            if (costofchromosomes[i] < min)
                min = costofchromosomes[i];
        }
        cout << "one most optimal solution : " << endl;
        //we do this to find the path
        for (int i = 0;i < 20;i++)
        {
            vector<int>TotalPath;
            int shortestpath_perchromosome = INT_MAX;
            vector<int>path;
            int aindex = 0;
            int indexarray[3];
            for (int j = 0;j < 4;j++)
            {
                int sidx = (population[i][j].x * m) + population[i][j].y;
                int didx = (population[i][j + 1].x * m) + population[i][j + 1].y;
                vector<int>parent;
                if (j >= 1 && j <= 3)
                {

                    indexarray[aindex] = sidx;
                    aindex++;
                }
                // Run Dijkstra's algorithm to find the shortest path
                Dijkstra dijkstra(graph, sidx, parent);
                int shortestPath = dijkstra.sumWeights[didx];


                path = getShortestPathandfindfitness(sidx, didx, parent, arr, shortestPath);
                if (j == 0)
                    TotalPath = path;
                else
                {
                    TotalPath.insert(TotalPath.end(), path.begin(), path.end());
                }
                // Print the shortest path cose
                if (shortestPath == INT_MAX)
                {
                    shortestpath_perchromosome = INT_MAX;
                    break;
                }
                if (j == 0)
                    shortestpath_perchromosome = shortestPath;
                else
                    shortestpath_perchromosome = shortestpath_perchromosome + shortestPath;


            }
            if (shortestpath_perchromosome == min)
            {
                print2dgrid(arr);
                cout << "the shortest path is : ";
                for (int i = 0; i < TotalPath.size(); i++)
                {

                    int ind1 = TotalPath[i] / 40;
                    int ind2 = TotalPath[i] % 40;
                    if (TotalPath[i] == indexarray[0] || TotalPath[i] == indexarray[1] || TotalPath[i] == indexarray[2])
                        arr[ind1][ind2].symbol = 'O';
                    else
                        arr[ind1][ind2].symbol = '#';

                    arr[start_ind1][start_ind2].symbol = 'S';
                    arr[destination_ind1][destination_ind2].symbol = 'D';
                    cout << "(" << ind1 << "," << ind2 << ")" << "  ";


                }
                arr[start_ind1][start_ind2].symbol = 'S';
                arr[destination_ind1][destination_ind2].symbol = 'D';

                cout << endl;
                print2dgrid(arr);
                cout << endl;
                break;
            }

        }



    }
    for (int i = 0; i < n; i++)
    {
        delete[] arr[i];
    }
    delete[] arr;
    return 0;
}


