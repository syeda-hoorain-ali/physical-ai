import { BrainIcon, CpuIcon, RocketIcon, ZapIcon } from "lucide-react";
import { AnimatedList } from "../ui/animated-list"


const listItems = [
  <div className="flex items-center gap-4 rounded-lg border border-border/50 bg-card/50 p-4 backdrop-blur">
    <CpuIcon className="h-8 w-8 text-primary" />
    <div>
      <h4 className="font-semibold text-foreground">Vision-Language-Action Models</h4>
      <p className="text-sm text-muted-foreground">Convergence of LLMs and Robotics</p>
    </div>
  </div>,

  <div className="flex items-center gap-4 rounded-lg border border-border/50 bg-card/50 p-4 backdrop-blur">
    <RocketIcon className="h-8 w-8 text-secondary" />
    <div>
      <h4 className="font-semibold text-foreground">Physical World Deployment</h4>
      <p className="text-sm text-muted-foreground">Real-world humanoid robot integration</p>
    </div>
  </div>,

  <div className="flex items-center gap-4 rounded-lg border border-border/50 bg-card/50 p-4 backdrop-blur">
    <ZapIcon className="h-8 w-8 text-primary" />
    <div>
      <h4 className="font-semibold text-foreground">Human-Robot Collaboration</h4>
      <p className="text-sm text-muted-foreground">Partnerships between people and robots</p>
    </div>
  </div>,

  <div className="flex items-center gap-4 rounded-lg border border-border/50 bg-card/50 p-4 backdrop-blur">
    <BrainIcon className="h-8 w-8 text-secondary" />
    <div>
      <h4 className="font-semibold text-foreground">Embodied Intelligence</h4>
      <p className="text-sm text-muted-foreground">Bridging digital brain and physical body</p>
    </div>
  </div>,
];

export const AnimatedListSection = () => {
  return (
    <section className="border-t border-border/30 bg-muted/20 py-24">
      <div className="container mx-auto px-4">
        <h2 className="mb-4 text-center text-3xl font-bold text-foreground md:text-4xl">
          What You'll Learn
        </h2>
        <p className="mx-auto mb-16 max-w-xl text-center text-muted-foreground">
          Key topics covered in this comprehensive textbook
        </p>

        <div className="mx-auto max-w-2xl">
          <AnimatedList items={listItems} delay={300} />
        </div>
      </div>
    </section>
  )
}
