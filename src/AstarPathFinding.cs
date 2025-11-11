struct Point
{
    public int X, Y;
    public Point(int x, int y)
    {
        X = x;
        Y = y;
    }
}

struct Node
{
    public int F, G, H; // F = G + H
    public Point Parent;
    public bool InOpenList, InClosedList;
}

struct PQNode
{
    public Point Point;
    public int Priority;
}

class PriorityQueue
{
    private List<PQNode> Data;

    public PriorityQueue()
    {
        Data = new List<PQNode>();
    }

    public void Push(Point point, int priority)
    {
        PQNode node = new PQNode();
        node.Point = point;
        node.Priority = priority;

        Data.Add(node);

        // Sort by Priority
        Data.Sort(delegate (PQNode a, PQNode b)
        {
            return a.Priority.CompareTo(b.Priority);
        });
    }

    public Point Pop()
    {
        if (Data.Count == 0)
            throw new InvalidOperationException("Queue is empty.");

        PQNode node = Data[0];
        Data.RemoveAt(0);
        return node.Point;
    }

    public bool IsEmpty()
    {
        return Data.Count == 0;
    }
}

class Pathfinding
{
    private readonly int[] dx = { -1, 1, 0, 0 };
    private readonly int[] dy = { 0, 0, -1, 1 };

    private int Heuristic(Point a, Point b)
    {
        return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
    }

    private bool IsValid(int x, int y, int m, int n, int[,] grid)
    {
        return x >= 0 && x < m && y >= 0 && y < n && grid[x, y] == 0;
    }

    public List<Point> AStar(int[,] grid, int m, int n, Point start, Point end)
    {
        PriorityQueue openList = new PriorityQueue();
        Node[,] nodes = new Node[m, n];
        List<Point> pathList = new List<Point>();

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                nodes[i, j].F = int.MaxValue;
                nodes[i, j].G = int.MaxValue;
                nodes[i, j].H = int.MaxValue;
                nodes[i, j].Parent = new Point(-1, -1);
                nodes[i, j].InOpenList = false;
                nodes[i, j].InClosedList = false;
            }
        }

        nodes[start.X, start.Y].G = 0;
        nodes[start.X, start.Y].H = Heuristic(start, end);
        nodes[start.X, start.Y].F = nodes[start.X, start.Y].H;

        openList.Push(start, nodes[start.X, start.Y].F);
        nodes[start.X, start.Y].InOpenList = true;

        while (!openList.IsEmpty())
        {
            Point current = openList.Pop();
            if (current.X == end.X && current.Y == end.Y)
            {
                Point path = end;
                while (path.X != -1 && path.Y != -1)
                {
                    pathList.Add(path);
                    path = nodes[path.X, path.Y].Parent;
                }

                pathList.Reverse();
                return pathList;
            }

            nodes[current.X, current.Y].InClosedList = true;

            for (int i = 0; i < 4; i++)
            {
                int nx = current.X + dx[i];
                int ny = current.Y + dy[i];

                if (IsValid(nx, ny, m, n, grid) && !nodes[nx, ny].InClosedList)
                {
                    int newG = nodes[current.X, current.Y].G + 1;

                    if (!nodes[nx, ny].InOpenList || newG < nodes[nx, ny].G)
                    {
                        nodes[nx, ny].G = newG;
                        nodes[nx, ny].H = Heuristic(new Point(nx, ny), end);
                        nodes[nx, ny].F = nodes[nx, ny].G + nodes[nx, ny].H;
                        nodes[nx, ny].Parent = current;

                        if (!nodes[nx, ny].InOpenList)
                        {
                            openList.Push(new Point(nx, ny), nodes[nx, ny].F);
                            nodes[nx, ny].InOpenList = true;
                        }
                    }
                }
            }
        }

        return pathList; // Return empty list if no path is found
    }
}

void followPath(List<Point> path)
{
    int curHead = 1; //start Heading

    for (int i = 1; i < path.Count; i++)
    {
        Point current = path[i - 1];
        Point next = path[i];

        // calculate next path
        int dx = next.X - current.X;
        int dy = next.Y - current.Y;
        
        int nextHead;
        if (dx == 1 && dy == 0) nextHead = 2; // ลง
        else if (dx == -1 && dy == 0) nextHead = 0; // ขึ้น
        else if (dx == 0 && dy == 1) nextHead = 1; // ขวา
        else if (dx == 0 && dy == -1) nextHead = 3; // ซ้าย
        else continue;

        // ปรับทิศทาง
        int turnSteps = (nextHead - curHead + 4) % 4; //calculate direction
        
        lcd("dx " + dx + " dy " + dy + " turn " + turnSteps);
        
        if (turnSteps == 1) turnR(); // right
        else if (turnSteps == 2) { turnR(); turnR(); } // U turn
        else if (turnSteps == 3) turnL(); // left

        // อัปเดตทิศทางปัจจุบัน
        curHead = nextHead;

        track();
        fd(100);
        sleep(500); 
        ao();
    }
    lcd("Finishh!!");
    ao();
}


void turnL()
{
    while (true)
    {
        sl(80);
        if (analog(1) < 512) break;
    }
    while (true)
    {
        sl(80);
        if (analog(2) < 512) break;
    }
    ao();
}

void turnR()
{
    while (true)
    {
        sr(80);
        if (analog(3) < 512) break;
    }
    while (true)
    {
        sr(80);
        if (analog(2) < 512) break;
    }
    ao();
}

void track(){
    while(true){
        if(analog(1)<512 && analog(3)>512){ //turnleft
           tl(100);
        } 
        else if(analog(1)>512 && analog(3)<512){ //turnright
            tr(100);
        }
        else if(analog(0)<512 && analog(4)<512){
            ao();
            break;   
        }
        else{ //fd
            fd(100);
        }
    }
}

void setup()
{
    // Grid (0=Path, 1=Wall)
    int[,] grid = new int[9, 9]
    {
        { 0, 1, 1, 1, 0, 0, 1, 0, 0 },
        { 0, 0, 0, 1, 0, 0, 0, 1, 0 },
        { 1, 1, 0, 1, 0, 0, 1, 0, 1 },
        { 0, 1, 0, 0, 0, 1, 1, 1, 1 },
        { 0, 0, 1, 1, 0, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0, 1, 0, 1 },
        { 0, 1, 1, 1, 1, 0, 1, 0, 1 },
        { 0, 1, 0, 0, 0, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 1, 0, 0 }
    };

    Point start = new Point(1, 1);
    Point end = new Point(5, 4);

    Pathfinding pathfinding = new Pathfinding();
    List<Point> path = pathfinding.AStar(grid, 9, 9, start, end);

    if (path.Count > 0)
    {
        lcd("Path found :      |SW| 1,2Path 3Run");
        int n=0;
        while(true){
            if(sw1()==0){
                n-=4;
                if(n<0) n=0;
                string Text = "|" + n/4 + "|";
                int finish = 0;
                if(n+4>path.Count) finish = path.Count;
                else finish = n+4;
                for(int i=n;i<finish;i++){
                    Text += "(" + path[i].X + "," + path[i].Y + ")";
                }
                lcd(Text);
                beep();
            }
            if(sw2()==0){
                int finish=0;
                if(n+4<path.Count){
                    n+=4;
                    if(n+4<=path.Count){   
                        finish = n+4;
                    }
                    if(n+4>path.Count){
                        finish = path.Count;
                    }
                    string Text = "|" + n/4 + "|";
                    for(int i=n;i<finish;i++){
                        Text += "(" + path[i].X + "," + path[i].Y + ")";
                    }
                    lcd(Text);
                }
                beep();
            }
            if(sw3()==0){
                sound(550,200);
                break;   
            }
        }
        followPath(path);
    }
    else
    {
        lcd("No path found.");
    }
}

void loop()
{
  
}