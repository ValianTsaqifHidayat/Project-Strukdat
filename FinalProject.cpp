#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_set>
#include <cstdint>
#include <limits>
#include <utility> 

const int WINDOW_WIDTH = 800; 
const int WINDOW_HEIGHT = 600; 
const double GRAVITY = 0.0;
const double TIMESTEP = 0.01; 
const double DAMPING = 0.999; 
const int NUM_BALLS = 67; 
const int QUAD_CAPACITY = 5; 

struct Vec2 {
    double x, y;
    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator*(double scalar) const { return Vec2(x * scalar, y * scalar); }
    Vec2 operator*(const Vec2& other) const { return Vec2(x * other.x, y * other.y); }
    Vec2 operator/(double scalar) const { 
        return (scalar != 0) ? Vec2(x / scalar, y / scalar) : Vec2(0, 0); 
    }

    double lengthSq() const { return x * x + y * y; }
    double length() const { return std::sqrt(lengthSq()); }
    Vec2 normalize() const {
        double len = length();
        return (len > 0) ? Vec2(x / len, y / len) : Vec2(0, 0);
    }
};

class Ball {
public:
    Vec2 current_pos;
    Vec2 old_pos;
    Vec2 acceleration;
    double radius;
    double mass;
    sf::CircleShape shape;

    Ball(Vec2 pos, Vec2 initial_vel, double r, double m, sf::Color color)
        : current_pos(pos), acceleration(0, 0), radius(r), mass(m) {
        
        old_pos = pos - initial_vel * TIMESTEP; 
        
        shape.setRadius((float)r);
        shape.setFillColor(color);
        shape.setOrigin((float)r, (float)r);
    }
    ~Ball() = default; 

    void update_position() {
        Vec2 velocity = current_pos - old_pos;
        Vec2 next_pos = current_pos + velocity * DAMPING + acceleration * (TIMESTEP * TIMESTEP);

        old_pos = current_pos;
        current_pos = next_pos;
        acceleration = Vec2(0, 0); 
        shape.setPosition((float)current_pos.x, (float)current_pos.y);
    }

    Vec2 get_velocity() const {
        return (current_pos - old_pos) / TIMESTEP;
    }

    void set_velocity(const Vec2& new_vel) {
        old_pos = current_pos - new_vel * TIMESTEP;
    }

    void constrain_bounds() {
        const double WALL_RESTITUTION = 0.95; 

        if (current_pos.x - radius < 0) {
            current_pos.x = radius;
            set_velocity(get_velocity() * Vec2(-WALL_RESTITUTION, 1.0)); 
        }
        else if (current_pos.x + radius > WINDOW_WIDTH) {
            current_pos.x = WINDOW_WIDTH - radius;
            set_velocity(get_velocity() * Vec2(-WALL_RESTITUTION, 1.0));
        }

        if (current_pos.y + radius > WINDOW_HEIGHT) {
            current_pos.y = WINDOW_HEIGHT - radius;
            set_velocity(get_velocity() * Vec2(1.0, -WALL_RESTITUTION));
        }
        else if (current_pos.y - radius < 0) {
            current_pos.y = radius;
            set_velocity(get_velocity() * Vec2(1.0, -WALL_RESTITUTION));
        }
    }
};

struct Boundary {
    Vec2 center;
    double half_width;
    double half_height;

    bool contains(const Ball* ball) const {
        return (ball->current_pos.x >= center.x - half_width &&
                ball->current_pos.x <= center.x + half_width &&
                ball->current_pos.y >= center.y - half_height &&
                ball->current_pos.y <= center.y + half_height);
    }

    bool intersects(const Boundary& range) const {
        return !(range.center.x - range.half_width > center.x + half_width ||
                 range.center.x + range.half_width < center.x - half_width ||
                 range.center.y - range.half_height > center.y + half_height ||
                 range.center.y + range.half_height < center.y - half_height);
    }
};

class Quadtree {
private:
    Boundary boundary;
    std::vector<Ball*> balls;
    bool divided;
    Quadtree* children[4]; 

public:
    Quadtree(const Boundary& bounds) : boundary(bounds), divided(false) {
        for (int i = 0; i < 4; ++i) children[i] = nullptr;
    }

    ~Quadtree() {
        for (int i = 0; i < 4; ++i) delete children[i];
    }

    void subdivide() {
        double x = boundary.center.x;
        double y = boundary.center.y;
        double hw = boundary.half_width / 2.0;
        double hh = boundary.half_height / 2.0;

        children[0] = new Quadtree({{x - hw, y - hh}, hw, hh}); 
        children[1] = new Quadtree({{x + hw, y - hh}, hw, hh}); 
        children[2] = new Quadtree({{x - hw, y + hh}, hw, hh}); 
        children[3] = new Quadtree({{x + hw, y + hh}, hw, hh}); 
        divided = true;
    }

    bool insert(Ball* ball) {
        if (!boundary.contains(ball)) return false;

        if (!divided && (int)balls.size() < QUAD_CAPACITY) {
            balls.push_back(ball);
            return true;
        }

        if (!divided) {
            subdivide();
            
            std::vector<Ball*> oldBalls = std::move(balls);
            balls.clear(); 
            for (Ball* b : oldBalls) {
                bool inserted = false;
                for (int i = 0; i < 4; ++i) {
                    if (children[i]->insert(b)) {
                        inserted = true;
                        break;
                    }
                }
                if (!inserted) {
                    balls.push_back(b);
                }
            }
        }

        for (int i = 0; i < 4; ++i) {
            if (children[i]->insert(ball)) return true;
        }

        balls.push_back(ball); 
        return true;
    }

    void query(const Boundary& range, std::vector<Ball*>& found_balls) {
        if (!boundary.intersects(range)) return;

        found_balls.insert(found_balls.end(), balls.begin(), balls.end());

        if (divided) {
            for (int i = 0; i < 4; ++i) {
                children[i]->query(range, found_balls);
            }
        }
    }
};

void resolve_collision(Ball* b1, Ball* b2) {
    Vec2 axis = b1->current_pos - b2->current_pos;
    double dist_sq = axis.lengthSq();
    double min_dist = b1->radius + b2->radius;
    double min_dist_sq = min_dist * min_dist;
    const double RESTITUTION = 0.95; 

    if (dist_sq < min_dist_sq && dist_sq > 0) {
        double dist = std::sqrt(dist_sq);
        double overlap = min_dist - dist;
        Vec2 n = axis.normalize();

        double total_mass = b1->mass + b2->mass;
        double m1_ratio = b2->mass / total_mass;
        double m2_ratio = b1->mass / total_mass;
        
        b1->current_pos = b1->current_pos + n * (overlap * m1_ratio);
        b2->current_pos = b2->current_pos - n * (overlap * m2_ratio);

        Vec2 v1 = b1->get_velocity();
        Vec2 v2 = b2->get_velocity();
        Vec2 v_rel = v1 - v2;
        double n_dot_vrel = v_rel.x * n.x + v_rel.y * n.y;

        if (n_dot_vrel < 0) {
            double j = (-(1.0 + RESTITUTION) * n_dot_vrel) / (1.0 / b1->mass + 1.0 / b2->mass);
            
            Vec2 impulse = n * j;
            
            Vec2 v1_new = v1 + impulse * (1.0 / b1->mass);
            Vec2 v2_new = v2 - impulse * (1.0 / b2->mass);
            
            b1->set_velocity(v1_new);
            b2->set_velocity(v2_new);
        }
    }
}


long long handle_collisions_bruteforce(std::vector<Ball*>& balls) {
    long long checks = 0;
    for (size_t i = 0; i < balls.size(); ++i) {
        for (size_t j = i + 1; j < balls.size(); ++j) {
            resolve_collision(balls[i], balls[j]);
            checks++;
        }
    }
    return checks;
}

long long handle_collisions_quadtree(std::vector<Ball*>& balls) {
    long long checks = 0;

    Boundary root_boundary = {{WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0}, WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0};
    Quadtree tree(root_boundary);
    for (Ball* ball : balls) {
        tree.insert(ball);
    }

    double max_radius = 0.0;
    for (Ball* b : balls) max_radius = std::max(max_radius, b->radius);

    std::unordered_set<uint64_t> checked;

    for (size_t i = 0; i < balls.size(); ++i) {
        Ball* b1 = balls[i];

        Boundary search_range = {b1->current_pos, b1->radius + max_radius, b1->radius + max_radius};

        std::vector<Ball*> candidates;
        tree.query(search_range, candidates); 

        for (Ball* b2 : candidates) {
            if (b1 == b2) continue;

            uintptr_t a = reinterpret_cast<uintptr_t>(b1);
            uintptr_t b = reinterpret_cast<uintptr_t>(b2);
            uintptr_t mn = std::min(a, b);
            uintptr_t mx = std::max(a, b);

            uint64_t key = (uint64_t)mn << 32 ^ (uint64_t)mx;

            if (checked.count(key)) continue;

            checked.insert(key);
            resolve_collision(b1, b2);
            checks++;
        }
    }

    return checks;
}


void initialize_balls(std::vector<Ball*>& balls, std::mt19937& gen, std::uniform_real_distribution<>& dis_pos, std::uniform_real_distribution<>& dis_rad, std::uniform_int_distribution<>& dis_color) {
    std::uniform_real_distribution<> dis_vel(-80.0, 80.0); 
    std::uniform_real_distribution<> dis_rad_new(15.0, 25.0); 

    for (int i = 0; i < NUM_BALLS; ++i) {
        double r = dis_rad_new(gen); 
        double m = r; 
        double px = dis_pos(gen);
        double py = dis_pos(gen);
        
        Vec2 initial_vel(dis_vel(gen), dis_vel(gen)); 

        px = std::max(r, std::min((double)WINDOW_WIDTH - r, px));
        py = std::max(r, std::min((double)WINDOW_HEIGHT - r, py));

        sf::Color random_color((sf::Uint8)dis_color(gen), (sf::Uint8)dis_color(gen), (sf::Uint8)dis_color(gen));

        balls.push_back(new Ball({px, py}, initial_vel, r, m, random_color)); 
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulasi Fisika: Inersia Lebih Santai");
    window.setFramerateLimit(60);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_pos(10.0, std::min((double)WINDOW_WIDTH, (double)WINDOW_HEIGHT) - 10.0);
    std::uniform_real_distribution<> dis_rad_unused(8.0, 18.0); 
    std::uniform_int_distribution<> dis_color(0, 255);

    std::vector<Ball*> balls;
    initialize_balls(balls, gen, dis_pos, dis_rad_unused, dis_color);

    bool use_quadtree = true; 

    sf::Clock clock;
    static const float fixed_update_time = (float)TIMESTEP; 
    static float accumulator = 0.0f;

    sf::Vector2i prev_window_pos = window.getPosition();

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space) {
                use_quadtree = !use_quadtree;
                std::cout << "Algoritma Diubah ke: " << (use_quadtree ? "Quadtree" : "Brute Force") << std::endl;
            }
        }

        sf::Time elapsed = clock.restart();
        accumulator += elapsed.asSeconds();

        sf::Vector2i current_window_pos = window.getPosition();
        
        Vec2 window_disp(
            (double)(current_window_pos.x - prev_window_pos.x),
            (double)(current_window_pos.y - prev_window_pos.y)
        );

        const double INERTIA_FACTOR = -2000.0; 
        Vec2 frame_acceleration = window_disp * (INERTIA_FACTOR * fixed_update_time); 
    
        prev_window_pos = current_window_pos;

        while (accumulator >= fixed_update_time) {
            
            for (Ball* ball : balls) { 
                Vec2 current_vel = ball->get_velocity();
                ball->set_velocity(current_vel + frame_acceleration); 

                ball->update_position();
                ball->constrain_bounds();
            }

            long long checks = 0;
            auto start = std::chrono::high_resolution_clock::now(); 

            if (use_quadtree) {
                checks = handle_collisions_quadtree(balls);
            } else {
                checks = handle_collisions_bruteforce(balls);
            }

            auto end = std::chrono::high_resolution_clock::now();
            double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
            
            window.setTitle("Simulasi | Algoritma: " + std::string(use_quadtree ? "Quadtree" : "Brute Force") +
                            " | Cek: " + std::to_string(checks) + " | Waktu: " + std::to_string(duration) + " ms");
            
            accumulator -= fixed_update_time;
        }

        window.clear(sf::Color::Black);
        for (const auto& ball : balls) {
            window.draw(ball->shape); 
        }
        window.display();
    }

    for (Ball* ball : balls) {
        delete ball; 
    }

    return 0;
}