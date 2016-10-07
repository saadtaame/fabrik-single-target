
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX_JOINTS 200
#define MAX_ITERS 50

typedef struct {
	float x, y, z;
} Joint;

Joint MakeJoint(float x, float y, float z) {
	Joint joint = {x, y, z};
	return joint;
}

void PrintJoint(Joint A) {
	fprintf(stdout, "[ %10f %10f %10f ]\n", A.x, A.y, A.z);
}

float Dist(Joint A, Joint B) {
	float dx = A.x - B.x;
	float dy = A.y - B.y;
	float dz = A.z - B.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

Joint Interpolate(Joint A, Joint B, float b) {
	float a = 1 - b;
	Joint joint = {a * A.x + b * B.x, a * A.y + b * B.y, a * A.z + b * B.z};
	return joint;
}

void Forward(Joint p[], float d[], int n, Joint t) {
	float lambda;
	int i;

	p[n - 1] = t;
	for(i = n - 2; i >= 0; i--) {
		lambda = d[i] / Dist(p[i], p[i + 1]);
		p[i] = Interpolate(p[i + 1], p[i], lambda);
	}
}

void Backward(Joint p[], float d[], int n, Joint t, Joint fixedJoint) {
	float lambda;
	int i;

	p[0] = fixedJoint;
	for(i = 0; i < n - 1; i++) {
		lambda = d[i] / Dist(p[i], p[i + 1]);
		p[i + 1] = Interpolate(p[i], p[i + 1], lambda);
	}
}

/* p[0] is root joint
   p[n - 1] is end effector
   t is target
 */
void FABRIK(Joint p[], int n, Joint t) {

	static float d[MAX_JOINTS];
	static const float TOL = 0.001;

	float totalLength;
	float lambda;

	int iters = 0;
	int i;

	Joint fixedJoint = p[0];

	totalLength = 0;
	for(i = 0; i < n - 1; i++) {
		d[i] = Dist(p[i], p[i + 1]);
		totalLength += d[i];
	}

	if(Dist(p[0], t) >= totalLength) {
		for(i = 0; i < n - 1; i++) {
			lambda = d[i] / Dist(p[i], t);
			p[i + 1] = Interpolate(p[i], t, lambda);
		}
	}
	else {
		while((iters++ < MAX_ITERS) && (Dist(p[n - 1], t) > TOL)) {
			Forward(p, d, n, t);
			Backward(p, d, n, t, fixedJoint);
		}
	}

	fprintf(stdout, "Iterations %d\n", iters);
	fprintf(stdout, "Joints:\n");
	for(i = 0; i < n; i++)
        PrintJoint(p[i]);
}

enum {
    SCREEN_HEIGHT = 480,
    SCREEN_WIDTH  = 640
};

int SDL_main(int argc, char *argv[]) {
    SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;
    SDL_Event event;
    int done = 0;
    int editMode = 1;

    Joint p[MAX_JOINTS];
	Joint t;
	int i, n = 0;

	SDL_Rect rect;
	rect.w = 7;
	rect.h = 7;

    if(SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        exit(EXIT_FAILURE);
    }
    atexit(SDL_Quit);

    window = SDL_CreateWindow(
        "Inverse Kinematics Demo",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    if(window == NULL) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        exit(EXIT_FAILURE);
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if(renderer == NULL) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        exit(EXIT_FAILURE);
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    while(done == 0) {
        if(SDL_WaitEvent(&event)) {
            switch(event.type) {
                case SDL_QUIT:
                    done = 1;
                    break;

                case SDL_KEYUP:
                    if(event.key.keysym.sym == SDLK_e)
                        editMode ^= 1;
                    else if(event.key.keysym.sym == SDLK_c) {
                        editMode = 1;
                        n = 0;
                    }
                    break;

                case SDL_MOUSEMOTION:
                    if(editMode == 0) {
                        fprintf(stdout, "Mouse at %d %d\n", event.motion.x, event.motion.y);
                        t = MakeJoint(event.motion.x, event.motion.y, 0);
                        FABRIK(p, n, t);
                    }
                    break;

                case SDL_MOUSEBUTTONUP:
                    if(editMode == 1) {
                        p[n++] = MakeJoint(event.button.x, event.button.y, 0);
                    }
                    break;

                default:
                    break;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 55, 155, 230, 255);
        for(i = 0; i < n - 1; i++)
            SDL_RenderDrawLine(renderer, p[i].x, p[i].y, p[i + 1].x, p[i + 1].y);
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        for(i =0; i < n; i++) {
            rect.x = p[i].x - 3;
            rect.y = p[i].y - 3;
            SDL_RenderDrawRect(renderer, &rect);
        }
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return EXIT_SUCCESS;
}
