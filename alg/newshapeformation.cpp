
#include "alg/newshapeformation.h"


NewShapeFormationParticle::NewShapeFormationParticle(const Node head,
                                               AmoebotSystem& system,
                                               const State state,
                                               const QString mode)
    : AmoebotParticle(head, -1, randDir(), system),
    _state(state),
    mode(mode),
    _parentDir(-1),
    _shapeDir(state == State::Seed ? 0 : -1) {}
//////////




void NewShapeFormationParticle::activate() {
    // alpha_1: idle or follower particles with a seed or retired neighbor become
    // roots and begin traversing the shape's surface.
    if (isContracted()
        && (_state == State::Idle || _state == State::Follower)
        && hasNbrInState({State::Seed, State::Retired})) {
        _parentDir = -1;
        _state = State::Root;
        _shapeDir = nextShapeDir(1);  // clockwise.
    }
    // alpha_2: idle particles with follower or root neighbors become followers
    // and join the spanning forest.
    else if (_state == State::Idle
             && hasNbrInState({State::Follower, State::Root})) {
        _parentDir = labelOfFirstNbrInState({State::Follower, State::Root});
        _state = State::Follower;
    }
    // alpha_3: contracted roots with no idle neighbors who are pointed at by a
    // retired or seed particle's construction direction retire.
    else if (isContracted()
             && _state == State::Root
             && !hasNbrInState({State::Idle})
             && canRetire()) {
        _shapeDir = nextShapeDir(-1);  // counter-clockwise.
        _state = State::Retired;
    }
    // alpha_4: contracted roots that can expand along the surface of the shape
    // do so.
    else if (isContracted()
             && _state == State::Root
             && !hasNbrAtLabel(_shapeDir)) {

        expand(_shapeDir);
    }

    // NEW ACTION -  root nodes that cannot expand due to being at a corner, and cannot retire will update direction to continue around shape
    else if (isContracted()
               && _state == State::Root
               && hasNbrAtLabel(_shapeDir)
               && (nbrAtLabel(_shapeDir)._state == State::Seed ||               nbrAtLabel(_shapeDir)._state == State::Retired)
               && !canRetire()) {

        _shapeDir = nextShapeDir(1);
    }
    // alpha_5: expanded followers and roots without idle neighbors but with a
    // tail child pull a tail child in a handover.
    else if (isExpanded()
             && (_state == State::Follower || _state == State::Root)
             && !hasNbrInState({State::Idle})
             && !conTailChildLabels().empty()) {
        if (_state == State::Root)
            _shapeDir = nextShapeDir(1);  // clockwise.
        int childLabel = conTailChildLabels()[0];
        nbrAtLabel(childLabel)._parentDir = dirToNbrDir(nbrAtLabel(childLabel),
                                                        (tailDir() + 3) % 6);
        pull(childLabel);
    }
    // alpha_6: expanded followers and roots without idle neighbors or tail
    // children contract their tails.
    else if (isExpanded()
             && (_state == State::Follower || _state == State::Root)
             && !hasNbrInState({State::Idle})
             && !hasTailChild()) {
        if (_state == State::Root)
            _shapeDir = nextShapeDir(1);  // clockwise.
        contractTail();
    }
}


int NewShapeFormationParticle::headMarkColor() const {
    switch(_state) {
    case State::Seed:      return 0x00ff00;
    case State::Idle:      return -1;
    case State::Follower:  return 0x0000ff;
    case State::Root:      return 0xff0000;
    case State::Retired:   return 0x000000;
    default:               return -1;
    }
}


int NewShapeFormationParticle::tailMarkColor() const {
    return headMarkColor();
}


int NewShapeFormationParticle::headMarkDir() const {
    if (_state == State::Idle) {
        return -1;
    } else if (_state == State::Follower) {
        return _parentDir;
    } else {  // State::Seed, State::Root, State::Retired.
        return _shapeDir;
    }
}


QString NewShapeFormationParticle::inspectionText() const {
    QString text;
    text += "Global Info:\n";
    text += "  head: (" + QString::number(head.x) + ", "
            + QString::number(head.y) + ")\n";
    text += "  orientation: " + QString::number(orientation) + "\n";
    text += "  globalTailDir: " + QString::number(globalTailDir) + "\n\n";
    text += "Local Info:\n";
    text += "  state: ";
    text += [this](){
        switch(_state) {
        case State::Seed:      return "seed\n";
        case State::Idle:      return "idle\n";
        case State::Follower:  return "follower\n";
        case State::Root:      return "root\n";
        case State::Retired:   return "retired\n";
        default:               return "no state\n";
        }
    }();
    text += "  parentDir: " + QString::number(_parentDir) + "\n";
    text += "  shapeDir: " + QString::number(_shapeDir) + "\n";
    text += "shape: ";
    text += [this](){
        if (mode == "h") {
            return "hexagon";
        } else if (mode == "s") {
            return "square";
        } else if (mode == "t1") {
            return "vertex triangle";
        } else if (mode == "t2") {
            return "center triangle";
        } else if (mode == "l") {
            return "line";
        } else {
            return "ERROR";
        }
    }();
    text += "\n";


    return text;
}
///////
NewShapeFormationParticle& NewShapeFormationParticle::nbrAtLabel(int label) const{
    return AmoebotParticle::nbrAtLabel<NewShapeFormationParticle>(label);
}


int NewShapeFormationParticle::labelOfFirstNbrInState(
    std::initializer_list<State> states, int startLabel) const {
    auto prop = [&](const NewShapeFormationParticle& p) {
        for (auto state : states) {
            if (p._state == state) {
                return true;
            }
        }
        return false;
    };


    return labelOfFirstNbrWithProperty<NewShapeFormationParticle>(prop, startLabel);
}


bool NewShapeFormationParticle::hasNbrInState(
    std::initializer_list<State> states) const {
    return labelOfFirstNbrInState(states) != -1;
}


int NewShapeFormationParticle::nextShapeDir(int orientation)  {
    // First, find a head label that points to a seed or retired neighbor.
    int shapeLabel;
    int nbrRetired;
    int pointingNbr;


    // Next, find the label that points along the shape's surface in a traversal
    // with the specified orientation. Perhaps counterintuitively, this means that
    // we search from the above label in the opposite orientation for the first
    // unoccupied or non-seed/retired neighbor.
    int numLabels = isContracted() ? 6 : 10;
    int numNextTo=0;

    if(orientation==1){
        for (int label : headLabels()) {
            if (hasNbrAtLabel(label)
                && (nbrAtLabel(label)._state == State::Seed
                    || nbrAtLabel(label)._state == State::Retired)) {
                shapeLabel = label;
            }
        }
        while (hasNbrAtLabel(shapeLabel)
               && (nbrAtLabel(shapeLabel)._state == State::Seed
                   || nbrAtLabel(shapeLabel)._state == State::Retired))
        {
            shapeLabel = (shapeLabel + orientation + numLabels) % numLabels;

        }

    }
    else if(orientation==-1){
        for (int label : headLabels()) {
            if (hasNbrAtLabel(label)
                && (nbrAtLabel(label)._state == State::Seed
                    || nbrAtLabel(label)._state == State::Retired)) {
                nbrRetired = label;
                if(pointsAtMe(nbrAtLabel(nbrRetired),nbrAtLabel(nbrRetired)._shapeDir)){
                    pointingNbr = nbrRetired;
                }
                if (mode=="l"){  //added so line goes both ways from seed
                    if(pointsAtMe(nbrAtLabel(nbrRetired),(nbrAtLabel(nbrRetired)._shapeDir+3)%6)){
                        pointingNbr = nbrRetired;
                    }
                }
                numNextTo= numNextTo+1;
            }
        }
        shapeLabel = nbrRetired;
        while (hasNbrAtLabel(shapeLabel)
               && (nbrAtLabel(shapeLabel)._state == State::Seed
                   || nbrAtLabel(shapeLabel)._state == State::Retired))
        {
            shapeLabel = (shapeLabel + orientation + numLabels) % numLabels;

        }
        if (mode == "t2") {

            if (numNextTo==2 && orientation==-1){
                shapeLabel = (shapeLabel + orientation + numLabels) % numLabels;
            }
        }
        else if (mode == "l"){
            shapeLabel = (pointingNbr +3) % numLabels;


        }
        else if (mode == "s"){
            if(orientation==-1){
                turnSignal = nbrAtLabel(pointingNbr).turnSignal;
                if(numNextTo ==2){
                    if(turnSignal==1){
                        shapeLabel = (shapeLabel + orientation + numLabels) % numLabels;
                        turnSignal =0;
                    }
                    else{turnSignal=1;}
                }

            }

        }
    }


    // Convert this label to a direction before returning.
    return labelToDir(shapeLabel);
}


bool NewShapeFormationParticle::canRetire() const {
    auto prop = [&](const NewShapeFormationParticle& p) {
        // originally:  return (p._state == State::Seed || p._state == State::Retired)  &&  (pointsAtMe(p, p._shapeDir))
        //changed so line formation goes both ways
        return (p._state == State::Seed || p._state == State::Retired)
               && ((mode=="l" &&  (pointsAtMe(p, p._shapeDir) ||(pointsAtMe(p,( p._shapeDir +3)%6) ) ) )|| (pointsAtMe(p, p._shapeDir)));
    };


    return labelOfFirstNbrWithProperty<NewShapeFormationParticle>(prop) != -1;
}


bool NewShapeFormationParticle::hasTailChild() const {
    auto prop = [&](const NewShapeFormationParticle& p) {
        return p._parentDir != -1
               && pointsAtMyTail(p, p.dirToHeadLabel(p._parentDir));
    };


    return labelOfFirstNbrWithProperty<NewShapeFormationParticle>(prop) != -1;
}


const std::vector<int> NewShapeFormationParticle::conTailChildLabels() const {
    std::vector<int> labels;
    for (int label : tailLabels())
        if (hasNbrAtLabel(label)
            && nbrAtLabel(label).isContracted()
            && nbrAtLabel(label)._parentDir != -1
            && pointsAtMyTail(nbrAtLabel(label), nbrAtLabel(label)._parentDir))
            labels.push_back(label);


    return labels;
}


NewShapeFormationSystem::NewShapeFormationSystem(int numParticles,
                                           double holeProb,
                                           QString mode) {
    // Insert the shape formation seed at (0,0).
    Q_ASSERT(mode == "h" || mode == "s" || mode == "t1" || mode == "t2" ||
             mode == "l");
    Q_ASSERT(numParticles > 0);
    Q_ASSERT(0 <= holeProb && holeProb <= 1);

    // Insert the seed at (0,0).
    std::set<Node> occupied;
    insert(new NewShapeFormationParticle(Node(0, 0), *this,
                                      NewShapeFormationParticle::State::Seed, mode));
    occupied.insert(Node(0, 0));

    std::set<Node> candidates;
    for (int i = 0; i < 6; ++i) {
        candidates.insert(Node(0, 0).nodeInDir(i));
    }

    // Add all other particles.
    int particlesAdded = 1;
    while (particlesAdded < numParticles && !candidates.empty()) {
        // Pick a random candidate node.
        int randIndex = randInt(0, candidates.size());
        Node randCand;
        for (auto cand = candidates.begin(); cand != candidates.end(); ++cand) {
            if (randIndex == 0) {
                randCand = *cand;
                candidates.erase(cand);
                break;
            } else {
                randIndex--;
            }
        }

        // With probability 1 - holeProb, add a new particle at the candidate node.
        if (randBool(1.0 - holeProb)) {
            insert(new NewShapeFormationParticle(randCand,  *this,
                                              NewShapeFormationParticle::State::Idle,
                                              mode));
            occupied.insert(randCand);
            particlesAdded++;

            // Add new candidates.
            for (int i = 0; i < 6; ++i) {
                if (occupied.find(randCand.nodeInDir(i)) == occupied.end()) {
                    candidates.insert(randCand.nodeInDir(i));
                }
            }
        }
    }
}


bool NewShapeFormationSystem::hasTerminated() const {
    for (auto p : particles) {
        auto hp = dynamic_cast<NewShapeFormationParticle*>(p);
        if (hp->_state != NewShapeFormationParticle::State::Seed
            && hp->_state != NewShapeFormationParticle::State::Retired)
            return false;
    }

    return true;
}

std::set<QString> NewShapeFormationSystem::getAcceptedModes() {
    std::set<QString> set = {"h", "t1", "t2", "s", "l"};
    return set;
}
