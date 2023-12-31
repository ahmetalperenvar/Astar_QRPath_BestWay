using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
namespace AStarAlgorithm
{
    public class Astar
    {
        List<Node> open_list;
        List<int> closed_list;
        public List<Node> best_way;
        public List<Node> updated_best_way; // Output of InsertMiddlePoints
        public class Node
        {
            public int id;
            public double x;
            public double y;
            public double h;
            public double g;
            public double cost;
            public Node parent;
            public List<int> neighbours;
        }
        double find_cost(Node source, Node destination,double neighbour_x,double neighbour_y)
        {
            double movement_cost = neighbour_x - source.x + neighbour_y - source.y;
            double heuristic_cost = Math.Pow(destination.x - neighbour_x, 2) + Math.Pow(destination.y - neighbour_y, 2);
            return movement_cost + heuristic_cost;
        }
        double calculate_g_cost(Node source,double neighbour_x, double neighbour_y)
        {
            double movement_cost = neighbour_x - source.x + neighbour_y - source.y;
            return movement_cost;
        }
        double calculate_h_cost(Node destination,double neighbour_x, double neighbour_y)
        {
            double heuristic_cost = Math.Pow(destination.x - neighbour_x, 2) + Math.Pow(destination.y - neighbour_y, 2);
            return heuristic_cost;
        }
        public bool isReached(double x,double y, Node destination)
        {
            if (x == destination.x && y == destination.y)
            {
                return true;
            }
            return false;
        }

        public int id2sequence(int which_id,ref List<CommonStack.symbolic_point> symbolic_point_list)
        {
            int output;
            output = 999;

            for (int i = 0; i < symbolic_point_list.Count(); i++)
            {
                if (symbolic_point_list[i].Id == which_id)
                {
                    output = i;
                    break;
                }
            }

            return output;
        }

        /*  Summary: finding all symbolic point between 2 given points. They should be in order according to their distances from startPoint. 
            Note: this algorithm also gives start_points itself and end point!*/
        private List<Node> find_symbolic_points_between_given_2_points(Node current, Node desired, ref List<CommonStack.symbolic_point> symbolic_point_list)
        {
            List<Node> symbolic_point_IDs   = new List<Node>();    // Output of the function
            List<Node> symbolic_points      = new List<Node>();
            symbolic_points.Add(current);
            symbolic_points.Add(desired);

            int length_of_symbolic_point_list = symbolic_point_list.Count;
            bool vertical = false;
            bool increasing = false;
            #region FIND POINTS BETWEEN TWO POINT 
            if (current.x == desired.x)
            {
                vertical = true;
                for (int i = 0; i < length_of_symbolic_point_list; i++)
                {
                    if (symbolic_point_list[i].Location.X == current.x)
                    {
                        if (current.y < desired.y)
                        {
                            increasing = true;
                            if (symbolic_point_list[i].Location.Y > current.y && symbolic_point_list[i].Location.Y < desired.y)
                            {
                                Node node_helper = new Node();
                                node_helper.id = symbolic_point_list[i].Id;
                                node_helper.x = symbolic_point_list[i].Location.X;
                                node_helper.y = symbolic_point_list[i].Location.Y;
                                //symbolic_points.Add(symbolic_point_list[i]);
                                symbolic_points.Add(node_helper);
                            }
                        }
                        else
                        {
                            increasing = false;
                            if (symbolic_point_list[i].Location.Y < current.y && symbolic_point_list[i].Location.Y > desired.y)
                            {
                                Node node_helper = new Node();
                                node_helper.id = symbolic_point_list[i].Id;
                                node_helper.x = symbolic_point_list[i].Location.X;
                                node_helper.y = symbolic_point_list[i].Location.Y;
                                //symbolic_points.Add(symbolic_point_list[i]);
                                symbolic_points.Add(node_helper);
                            }
                        }
                    }
                }
            }
            else if (current.y == desired.y)
            {
                vertical = false;
                for (int i = 0; i < length_of_symbolic_point_list; i++)
                {
                    if (symbolic_point_list[i].Location.Y == current.y)
                    {
                        if (current.x < desired.x)
                        {
                            increasing = true;
                            if (symbolic_point_list[i].Location.X > current.x && symbolic_point_list[i].Location.X < desired.x)
                            {
                                Node node_helper = new Node();
                                node_helper.id = symbolic_point_list[i].Id;
                                node_helper.x = symbolic_point_list[i].Location.X;
                                node_helper.y = symbolic_point_list[i].Location.Y;
                                //symbolic_points.Add(symbolic_point_list[i]);
                                symbolic_points.Add(node_helper);
                            }
                        }
                        else
                        {
                            increasing = false;
                            if (symbolic_point_list[i].Location.X < current.x && symbolic_point_list[i].Location.X > desired.x)
                            {
                                Node node_helper = new Node();
                                node_helper.id = symbolic_point_list[i].Id;
                                node_helper.x = symbolic_point_list[i].Location.X;
                                node_helper.y = symbolic_point_list[i].Location.Y;
                                //symbolic_points.Add(symbolic_point_list[i]);
                                symbolic_points.Add(node_helper);
                            }
                        }
                    }
                }
            }
            #endregion

            #region SORT SYMBOLIC POINTS BETWEEN THEM
            int length_symbolic_points = symbolic_points.Count();
            if (vertical)
            {
                if (increasing)
                {
                    sorting_symbolic_point(ref symbolic_points, length_symbolic_points, true, true);
                }
                else
                {
                    sorting_symbolic_point(ref symbolic_points, length_symbolic_points, true, false);
                }
            }
            else
            {
                if (increasing)
                {
                    sorting_symbolic_point(ref symbolic_points, length_symbolic_points, false, true);
                }
                else
                {
                    sorting_symbolic_point(ref symbolic_points, length_symbolic_points, false, false);
                }
            }
            #endregion

            for (int i = 0; i < symbolic_points.Count; i++)
            {
                symbolic_point_IDs.Add(symbolic_points[i]);
            }
            
            return symbolic_point_IDs;
        }


        private void sorting_symbolic_point(ref List<Node> input, int length, bool vertical, bool low_high)
        {
            double current_val;
            int j;

            if (vertical)
            {
                if (low_high)
                {
                    for (int i = 0; i < length; i++)
                    {
                        Node temp = input[i];
                        current_val = input[i].y;
                        j = i - 1;

                        for (; j >= 0 && current_val < input[j].y; j--)
                        {
                            input[j + 1] = input[j];
                        }
                        input[j + 1] = temp;

                    }
                }
                else
                {
                    for (int i = 0; i < length; i++)
                    {
                        Node temp = input[i];
                        current_val = input[i].y;
                        j = i - 1;

                        for (; j >= 0 && current_val > input[j].y; j--)
                        {
                            input[j + 1] = input[j];
                        }
                        input[j + 1] = temp;

                    }
                }
            }
            else
            {
                if (low_high)
                {
                    for (int i = 0; i < length; i++)
                    {
                        Node temp = input[i];
                        current_val = input[i].x;
                        j = i - 1;

                        for (; j >= 0 && current_val < input[j].x; j--)
                        {
                            input[j + 1] = input[j];
                        }
                        input[j + 1] = temp;
                    }
                }
                else
                {
                    for (int i = 0; i < length; i++)
                    {
                        Node temp = input[i];
                        current_val = input[i].x;
                        j = i - 1;

                        for (; j >= 0 && current_val > input[j].x; j--)
                        {
                            input[j + 1] = input[j];
                        }
                        input[j + 1] = temp;
                    }
                }
            }
        }


        private bool InsertMiddlePoints(ref List<CommonStack.symbolic_point> _symbolic_point_list)
        {
            updated_best_way = new List<Node>();
            for (int i = 0; i < best_way.Count -1 ; i++)
            {
                List<Node> result = find_symbolic_points_between_given_2_points(best_way[i], best_way[i + 1], ref _symbolic_point_list);

                if (result.Count < 2)
                {
                    CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Error, "InsertMiddlePoints",
                                                "Illogical stament! Error: result.Count < 2 because  result.Count is " + result.Count.ToString() );
                    return false;
                }
                else if(result.Count >= 2 )
                {
                    for (int j = 0; j < result.Count - 1; j++)
                    {
                        updated_best_way.Add(result[j]); 
                    }
                }
                else
                {
                    //Illogical!
                    CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Error, "InsertMiddlePoints",
                            "Illogical stament! Error is    entered else");
                    return false;

                }
            }

            int best_way_count = best_way.Count; 
            updated_best_way.Add(best_way[best_way_count-1]);
            return true;

        }


        public bool FindBestWay(Node source,Node destination, ref List<CommonStack.symbolic_point> symbolic_point_list)
        {
            if(source.id == destination.id)
            {
                CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Error, "FindBestWay",
                    "source and target are same");
                return false; 
            }
            // Reached but not preferred nodes list
            open_list = new List<Node>();
            // Reached preferred and visited nodes 
            closed_list = new List<int>(); 
            // Current node's neighbours cost list to choose minimum one 
            List<double> neighbour_costs ;
            // Creating node for start point 
            Node current = new Node();
            // Adding start point to openlist
            open_list.Add(source);
            //searching process is continiue while open_list.count != 0 and
            // while destination point is unreached
            while (open_list.Count>0)
            {
                // Sorting open list to choose best node has minimum cost 
                open_list = open_list.OrderBy(x => x.cost).ToList();
                // Assign the least cost one of open list to current 
                current = open_list[0];
                // Remove the chosen point from open list to continiue searching process with 
                open_list.Remove(current);
                // Add current ones id to visiteds list
                closed_list.Add(current.id);
                // Assign max value to uncalculated cost to avoid to confuse of choosing min cost
                double min_cost = double.MaxValue;
                // To avoid unassigned variable bug because it can be anything in c++
                int min_cost_index = -1;
                // Assign new list of neighbour_cost list count of current node's neighbours 
                neighbour_costs = new List<double>(current.neighbours.Count);
                // Keep searching through current node's count
                for (int i = 0; i < current.neighbours.Count; i++)
                {
                    // Check if the neighbour has been visited before to avoid loop
                    if (!closed_list.Contains( current.neighbours[i]))
                    {
                        double neighbour_x = symbolic_point_list[id2sequence(current.neighbours[i], ref symbolic_point_list)].Location.X;
                        double neighbour_y = symbolic_point_list[id2sequence(current.neighbours[i], ref symbolic_point_list)].Location.Y;
                        // Check if position of neighbour's neighbour is equal to the destination position   
                        if (isReached(neighbour_x, neighbour_y, destination))
                        {
                            // Assign new node list to best_way 
                            best_way = new List<Node>();
                            // Assign the recent node as the parent of the destination node
                            destination.parent = current;
                            // Add the destination node to best_way list 
                            // The list starts from the destination and goes to start point
                            best_way.Add(destination);

                            if (current.parent == null) 
                            {
                                // It means destination node is one neighbour of source node
                                best_way = new List<Node>();
                                best_way.Add(current);
                                best_way.Add(destination);
                                CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Debug, "FindBestWay",
                                    "destination node is one neighbour of source node");
                                
                                
                                // To Fix DEBUG
                                if (InsertMiddlePoints(ref symbolic_point_list))
                                {
                                    CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Debug, "FindBestWay",
                                        "The path is created successfully.");
                                    return true;

                                }
                                else
                                {
                                    CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Error, "FindBestWay",
                                        "Some problem occured in previous function!");
                                    return false;
                                }
                            }
                            // Keep going while the parrent node is equal to start point node 
                            while (current.parent != source)
                            {
                                best_way.Add(current);
                                current = current.parent;
                            }
                            best_way.Add(current);
                            best_way.Add(source);
                            // Reverse the best_way list to make it in order from source to destination
                            best_way.Reverse();
                            if (InsertMiddlePoints(ref symbolic_point_list))
                            {
                                CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Debug, "FindBestWay",
                                    "The path is created successfully."); 
                                return true;
                           
                            }
                            else
                            {
                                CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Error, "FindBestWay",
                                    "Some problem occured in previous function!");
                                return false;
                            }
                            
                        }
                        // Creating node for current neighbour and add its neighbours from symbolic_point_list 
                        Node node = new Node();
                        node.x = neighbour_x;
                        node.y = neighbour_y;
                        node.neighbours = symbolic_point_list[id2sequence(current.neighbours[i], ref symbolic_point_list)].GiveAllNeighbours();

                        node.id = symbolic_point_list[id2sequence(current.neighbours[i], ref symbolic_point_list)].Id;
                        // Calculating cost of the current neighbour 
                        node.cost = current.g + calculate_g_cost(current,neighbour_x,neighbour_y) + calculate_h_cost( destination, neighbour_x, neighbour_y);
                        // Assign the parent node of the neighbour 
                        node.parent = current;
                        // Add the node to open list to keep searching 
                        open_list.Add(node);
                    }

   
                }


            }
            CommonStack.LogAndMessage.WriteWithVerbosity(CommonStack.MessageVerbosityLevel.Error, "FindBestWay",
                "There is no proper path found."); 
            return false;
        }

    }
}