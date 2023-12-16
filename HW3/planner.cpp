#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <chrono>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    void not_true()
    {
        this->truth = !this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions() const
    {
        return this->actions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() const
    {
        return this->goal_conditions;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> G_Preconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> G_Effects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    GroundedAction(string name, list<string> arg_values,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> G_Pre,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> G_Eff)
    {
        this->name = name;
        for (string ar : arg_values)
            this->arg_values.push_back(ar);
        for (GroundedCondition gc : G_Pre)
            this->G_Preconditions.insert(gc);
        for (GroundedCondition gc : G_Eff)
            this->G_Effects.insert(gc);
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions()
    {
        return this->G_Preconditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects()
    {
        return this->G_Effects;
    }


    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

struct Node
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;
    double f;
    double g;
    double h;

    int parent_id= -1;
    string parent_state = "";
};

class Planner
{
    private:
        vector<GroundedAction> Gactions_precomp;
        unordered_set<Action, ActionHasher, ActionComparator> actions_precomp;
        vector<string> symbols_all;

        Env* env;
        int num_symbols = 0;

        vector<vector<string>> perm;
        vector<vector<string>> comb;
        vector<string> tempcomb;
        string init_node;

    public:
        stack<GroundedAction> path;
        int count = 0;

        Planner(Env* env)
        {
            this->env = env;
        }

        void precomp();
        void get_Gactions(Action &, vector<vector<string>> &);
        void astar();
        void get_sym_comb(int, int);
        void get_sym_perms(vector<string> &);
        bool goal(Node &);
        void backtrack(string &, unordered_map<string, Node>&);
        double heuristics(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &, int);
        string hash_states(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &);
        double empty_delete_heuristics(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &);
};

double Planner::heuristics(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &curr_node, int lit)
{
    double heu = 0;
    for (GroundedCondition gc : env->get_goal_conditions())
    {
        if(curr_node.find(gc) == curr_node.end())
        {
            ++heu;
        }
    }
    return (heu/lit);
    // return heu;
}

string Planner::hash_states(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &node_state)
{
    set<string> state;
    string hashed = "";

    for (GroundedCondition gc : node_state)
    {
        state.insert(gc.toString());
    }
    for (auto it = state.begin(); it != state.end(); it++)
    {
        hashed += *it;
    }
    return hashed;
}

bool Planner::goal(Node &curr_node)
{
    for (GroundedCondition gc : env->get_goal_conditions())
    {
        if (curr_node.state.find(gc) == curr_node.state.end())
        {
            return 0;
        }
    }
    return 1;
}

void Planner::backtrack(string &goal_node, unordered_map<string, Node> &graph)
{
    string curr_node = goal_node;
    while(curr_node != init_node)
    {
        path.push(Gactions_precomp[graph[curr_node].parent_id]);
        curr_node = graph[curr_node].parent_state;
    }
    return;
}

void Planner::get_sym_comb(int offset, int i)
{
    //cout << "here" << endl;
    if (i == 0)
    {
        comb.push_back(tempcomb);
        return;
    }
    for (int j = offset; j <= num_symbols - i; ++j)
    {
        tempcomb.push_back(symbols_all[j]);
        get_sym_comb(j+1, i-1);
        tempcomb.pop_back();
    }
}

void Planner::get_sym_perms(vector<string> &c)
{
    sort(c.begin(), c.end());
    do
    {
        perm.push_back(c);
    }while (next_permutation(c.begin(), c.end()));
}

void Planner::precomp()
{
    actions_precomp = env->get_actions();
    vector<string> symbols_temp(env->get_symbols().begin(), env->get_symbols().end());
    symbols_all = symbols_temp;

    num_symbols = symbols_all.size();
    int arguments = 0;

    for (Action action : actions_precomp)
    {
        arguments = action.get_args().size();
        get_sym_comb(0, arguments);
        for (int i = 0; i<comb.size(); i++)
        {
            get_sym_perms(comb[i]);
        }

        // cout << "All Combinations" << endl;
        // for (int k=0; k<comb.size(); k++)
        // {
        //     for (int j = 0; j<comb[k].size(); j++)
        //     {
        //         cout<< comb[k][j] <<" ";
        //     }
        //     cout << "\n";
        // }
        // cout << "All Permutations" << endl;
        // for (int k=0; k<perm.size(); k++)
        // {
        //     for (int j = 0; j<perm[k].size(); j++)
        //     {
        //         cout<< perm[k][j] <<" ";
        //     }
        //     cout << "\n";
        // }

        get_Gactions(action, perm);

        comb.clear();
        tempcomb.clear();
        perm.clear();
    }
}

void Planner::get_Gactions(Action &action, vector<vector<string>> &args)
{
    //Preconditions and effects of this action
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions = action.get_preconditions();
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects = action.get_effects();

    //for each permutation of actual symbols
    for (vector<string> argument : args) 
    {
        list<string> GA_argument (argument.begin(), argument.end()); //A,B,C

        // cout << "Arguments in List:\n";
        // list<string>::const_iterator it;
        // for (it = GA_argument.begin(); it!= GA_argument.end(); ++it)
        // {
        //     cout << *it << " ";
        // }
        // cout << "\n";

        //building a look-up table - to replace placeholders in 
        //the action's conditions with the actual arguments
        unordered_map<string, string> argument_lookup;

        //get the placeholder arguments
        list<string> act_arg = action.get_args();  //b,x,y

        //map the placeholder argument with the actual symbol
        list<string>::const_iterator act_it, GA_it;
        for (act_it = act_arg.begin(), GA_it = GA_argument.begin(); act_it != act_arg.end(); ++act_it, ++GA_it)
        {
            argument_lookup[*act_it] = *GA_it;
        }

        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> G_preconditions;
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> G_effects;

        //For each precondition of this action
        for (Condition pre : preconditions)
        {
            list<string> GC_args;
            list<string> pre_args = pre.get_args(); //get the placeholder arguments of this precondition

            list<string>::const_iterator pre_it;
            for (pre_it = pre_args.begin(); pre_it != pre_args.end(); ++pre_it) //iterate through each arguemnt of a precondition
            {
                if(argument_lookup[*pre_it] == "")
                {
                    GC_args.push_back(*pre_it);
                }
                else
                {
                    GC_args.push_back(argument_lookup[*pre_it]); //store the symbol associated with this key value
                }
            }
            GroundedCondition gc(pre.get_predicate(), GC_args, pre.get_truth());  //create a grounded precondition
            G_preconditions.insert(gc); //add to the set of all Grounded preconditions
        }

        // cout<<"Grounded Preconds:\n";
        // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it_gpre;
        // for(it_gpre = G_preconditions.begin(); it_gpre != G_preconditions.end(); ++it_gpre)
        // {
        //     cout<<*it_gpre<<",";
        // }
        // cout<<"\n";

        //For each effects of this action
        for (Condition eff : effects)
        {
            list<string> GC_args_eff;
            list<string> eff_args = eff.get_args();
            
            list<string>::const_iterator eff_it;
            for (eff_it = eff_args.begin(); eff_it != eff_args.end(); ++eff_it)
            {
                if(argument_lookup[*eff_it] == "")
                {
                    GC_args_eff.push_back(*eff_it);
                }
                else
                {
                    GC_args_eff.push_back(argument_lookup[*eff_it]);
                }
            }
            GroundedCondition gc(eff.get_predicate(), GC_args_eff, eff.get_truth());
            G_effects.insert(gc);
        }

        // cout<<"Grounded Effects:\n";
        // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it_geff;
        // for(it_geff = G_effects.begin(); it_geff != G_effects.end() ; ++it_geff)
        // {
        //     cout<<*it_geff<<",";
        // }
        // cout<<"\n";

        GroundedAction ga(action.get_name(), GA_argument, G_preconditions, G_effects);
        Gactions_precomp.push_back(ga);

        // cout <<"Grounded Actions:\n";
        // for (const GroundedAction& ga : Gactions_precomp) {
        //     std::cout << ga.toString() << std::endl;
        // }

        // // Print Preconditions
        // std::cout << "Preconditions:" << std::endl;
        // for (const GroundedCondition& gc : ga.get_preconditions()) {
        //     std::cout << "  - " << gc.toString() << std::endl;
        // }

        // // Print Effects
        // std::cout << "Effects:" << std::endl;
        // for (const GroundedCondition& gc : ga.get_effects()) {
        //     std::cout << "  - " << gc.toString() << std::endl;
        // }
        // cout<<"\n";
    }
}

double Planner::empty_delete_heuristics(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &curr_node_heur)
{
    unordered_map<string, bool> closedlist;
    unordered_map<string, Node> graph_heur;
    std::priority_queue<std::pair<double, string>, vector<std::pair<double, string>>, std::greater<std::pair<double, string>>> openlist;

    Node init_node;
    init_node.state = curr_node_heur;
    string init_node_hash = hash_states(init_node.state);
    init_node.g = 0;
    init_node.f = 0;
    init_node.h = 0;
    graph_heur[init_node_hash] = init_node;
    openlist.push(make_pair(init_node.f, init_node_hash));

    while (!openlist.empty())
    {
        pair<double, string> curr_node = openlist.top();
        openlist.pop();

        if (closedlist[curr_node.second] == true)
        {
            continue;
        }
        closedlist[curr_node.second] = true;
        Node heur_node = graph_heur[curr_node.second];

        if (goal(heur_node))
        {
            double path_length = 0;
            while (curr_node.second != init_node_hash)
            {
                ++path_length;
                curr_node.second = graph_heur[curr_node.second].parent_state;
            }
            // cout << "pathlength: " << path_length << endl;
            return path_length;  
        }

        bool valid_heur = 1;
        int num_actions_heur = -1;
        string hashed_new_heur;
        int literals_heur = 0;

        //iterate through all the precomputed grounded actions
        for (GroundedAction ga : this->Gactions_precomp)
        {
            // std::cout << ga.toString() << std::endl;
            //increment the counter for every grounded action
            num_actions_heur++;
            // cout << "actions: " << num_actions << endl;
            valid_heur = 1;

            //for every grounded action, check the preconditions and whether the current state satisfies it or not
            for (GroundedCondition pre_gc : ga.get_preconditions())
            {
                if (heur_node.state.find(pre_gc) == heur_node.state.end())
                {
                    valid_heur = 0;
                    break;
                }
            }

            literals_heur = 0;

            if (valid_heur)
            {
                Node newNode_heur;

                newNode_heur.state = heur_node.state;
                for (GroundedCondition eff : ga.get_effects())
                {
                    // cout << "eff: " << eff << endl;
                    if (eff.get_truth())
                    {
                        newNode_heur.state.insert(eff);
                        literals_heur++;
                        count++;
                    }
                }

                hashed_new_heur = hash_states(newNode_heur.state);
                if (closedlist[hashed_new_heur])
                {
                    continue;
                }

                newNode_heur.g = heur_node.g + 1;

                double heu = 0;
                for (GroundedCondition heur_gc : env->get_goal_conditions())
                {
                    if (newNode_heur.state.find(heur_gc) == newNode_heur.state.end())
                    {
                        ++heu;
                    }
                }

                newNode_heur.h = heu/literals_heur;

                newNode_heur.f = newNode_heur.g + newNode_heur.h;

                if (graph_heur.find(hashed_new_heur) == graph_heur.end() || newNode_heur.g < graph_heur[hashed_new_heur].g)
                {
                    newNode_heur.parent_id = num_actions_heur;
                    newNode_heur.parent_state = curr_node.second;
                    graph_heur[hashed_new_heur] = newNode_heur;
                    openlist.push(make_pair(newNode_heur.f, hashed_new_heur));
                }
            }
        }
    }
    return 0;
}

void Planner::astar()
{
    unordered_map<string, bool> CLOSEDLIST;
    std::priority_queue<std::pair<double, string>, vector<std::pair<double, string>>, std::greater<std::pair<double, string>>> OPENLIST;
    unordered_map<string, Node> graph;

    Node startNode;

    //get the initial state and add to the graph
    startNode.state = this->env->get_initial_conditions();
    init_node = hash_states(startNode.state); //hashing states - which act as the coordinates of the states in the graph
    startNode.g = 0;
    startNode.f = 0;
    graph[init_node] = startNode;

    //add it to the openlist
    OPENLIST.push(make_pair(startNode.f, init_node));

    while(!OPENLIST.empty())
    {
        //pop a node with lowest f value from the openlist
        pair<double, string> curr_node = OPENLIST.top();
        OPENLIST.pop();

        if(CLOSEDLIST[curr_node.second] == true)
        {
            continue;
        }

        //add the node to the openlist if not already added
        CLOSEDLIST[curr_node.second] = true;

        Node node = graph[curr_node.second];

        //check if goal
        if (goal(node))
        {
            // cout << "Goal found" << endl;
            cout << "Number of states expanded: " << CLOSEDLIST.size() << endl;
            // cout << "parent id: " << node.parent_id << endl;
            // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it;
            // for (it = node.state.begin(); it != node.state.end(); ++it)
            // {
            //     cout << *it << ",";
            // }
            // cout<<'\n';
            backtrack(curr_node.second, graph);
            return;
        }

        bool valid = 1;
        int num_actions = -1;
        string hashed_new;
        int literals = 0;

        //iterate through all the precomputed grounded actions
        for (GroundedAction ga : this->Gactions_precomp)
        {
            // std::cout << ga.toString() << std::endl;
            //increment the counter for every grounded action
            num_actions++;
            // cout << "actions: " << num_actions << endl;
            valid = 1;

            //for every grounded action, check the preconditions and whether the current state satisfies it or not
            for (GroundedCondition pre_gc : ga.get_preconditions())
            {
                if (node.state.find(pre_gc) == node.state.end())
                {
                    valid = 0;
                    break;
                }
            }

            literals = 0;

            if (valid)
            {
                // cout << "Grounded Action: " << endl;
                // std::cout << ga.toString() << std::endl;

                // cout << "valid action id: " << num_actions << endl;
                Node newNode;

                newNode.state = node.state;
                for (GroundedCondition eff : ga.get_effects())
                {
                    if (eff.get_truth())
                    {
                        newNode.state.insert(eff);
                        literals++;
                    }
                    else
                    {
                        eff.not_true();
                        newNode.state.erase(newNode.state.find(eff));
                    }
                }
                // cout << "literals of this action: " << literals << endl;

                // cout << "New state:" << endl;
                // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>::const_iterator it;
                // for (it = newNode.state.begin(); it != newNode.state.end(); ++it)
                // {
                //     cout << *it << ",";
                // }
                // cout<<'\n';

                hashed_new = hash_states(newNode.state);
                if (CLOSEDLIST[hashed_new])
                {
                    continue;
                }

                newNode.g = node.g + 1;
                
//////////////////////////////////////////////////////////////////////////////////////////////////////////
                //HEURISTICS

                //No heuristics
                // newNode.h = 0;

                //with heuristics
                newNode.h = heuristics(newNode.state, literals);

                //empty-delete-list heuristic
                // newNode.h = empty_delete_heuristics(newNode.state);
                // cout << "size: " << count << endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////

                newNode.f = newNode.g + newNode.h;

                if (graph.find(hashed_new) == graph.end() || newNode.g < graph[hashed_new].g)
                {
                    newNode.parent_id = num_actions;
                    newNode.parent_state = curr_node.second;
                    graph[hashed_new] = newNode;
                    OPENLIST.push(make_pair(newNode.f, hashed_new));
                }
            }
        }
    }
}

list<GroundedAction> planner(Env* env)
{
    // TODO: INSERT YOUR PLANNER HERE

    // Blocks World example (CHANGE THIS)
    // cout << endl << "CREATING DEFAULT PLAN" << endl;/
    list<GroundedAction> actions;
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    Planner plan(env);
    auto start = std::chrono::high_resolution_clock::now();
    plan.precomp();
    plan.astar();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time taken by Planner: " << duration.count() << " milliseconds" << std::endl;

    while(!plan.path.empty())
    {
        actions.push_back(plan.path.top());
        plan.path.pop();
    }

    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}