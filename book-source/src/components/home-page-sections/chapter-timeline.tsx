import { BookOpen, Cpu, Box, Layers, Bot, MessageSquare, GraduationCap } from "lucide-react";

const chapters = [
  {
    weeks: "Weeks 1-2",
    title: "Introduction to Physical AI",
    icon: <BookOpen className="h-6 w-6" />,
    topics: [
      "Foundations of Physical AI and embodied intelligence",
      "From digital AI to robots that understand physical laws",
      "Overview of humanoid robotics landscape",
      "Sensor systems: LIDAR, cameras, IMUs, force/torque sensors",
    ],
  },
  {
    weeks: "Weeks 3-5",
    title: "ROS 2 Fundamentals",
    icon: <Cpu className="h-6 w-6" />,
    topics: [
      "ROS 2 architecture and core concepts",
      "Nodes, topics, services, and actions",
      "Building ROS 2 packages with Python",
      "Launch files and parameter management",
    ],
  },
  {
    weeks: "Weeks 6-7",
    title: "Robot Simulation with Gazebo",
    icon: <Box className="h-6 w-6" />,
    topics: [
      "Gazebo simulation environment setup",
      "URDF and SDF robot description formats",
      "Physics simulation and sensor simulation",
      "Introduction to Unity for robot visualization",
    ],
  },
  {
    weeks: "Weeks 8-10",
    title: "NVIDIA Isaac Platform",
    icon: <Layers className="h-6 w-6" />,
    topics: [
      "NVIDIA Isaac SDK and Isaac Sim",
      "AI-powered perception and manipulation",
      "Reinforcement learning for robot control",
      "Sim-to-real transfer techniques",
    ],
  },
  {
    weeks: "Weeks 11-12",
    title: "Humanoid Robot Development",
    icon: <Bot className="h-6 w-6" />,
    topics: [
      "Humanoid robot kinematics and dynamics",
      "Bipedal locomotion and balance control",
      "Manipulation and grasping with humanoid hands",
      "Natural human-robot interaction design",
    ],
  },
  {
    weeks: "Week 13",
    title: "Conversational Robotics",
    icon: <MessageSquare className="h-6 w-6" />,
    topics: [
      "Integrating GPT models for conversational AI in robots",
      "Speech recognition and natural language understanding",
      "Multi-modal interaction: speech, gesture, vision",
    ],
  },
  {
    weeks: "Assessments",
    title: "Course Projects",
    icon: <GraduationCap className="h-6 w-6" />,
    topics: [
      "ROS 2 package development project",
      "Gazebo simulation implementation",
      "Isaac-based perception pipeline",
      "Capstone: Simulated humanoid robot with conversational AI",
    ],
  },
];

export const ChapterTimeline = () => {
  return (
    <section className="py-24 border-t border-border/30">
      <div className="container mx-auto px-4">
        <h2 className="mb-4 text-center text-3xl font-bold text-foreground md:text-4xl">
          Book Chapters
        </h2>
        <p className="mx-auto mb-16 max-w-xl text-center text-muted-foreground">
          A comprehensive 13-week journey through Physical AI and Humanoid Robotics
        </p>

        <div className="relative">
          {/* Center line */}
          <div className="absolute left-4 md:left-1/2 top-0 bottom-0 w-0.5 bg-linear-to-b from-primary via-secondary to-primary md:-translate-x-1/2" />

          <div className="space-y-12">
            {chapters.map((chapter, index) => {
              const isLeft = index % 2 === 0;
              
              return (
                <div
                  key={index}
                  className={`relative flex items-start gap-8 ${
                    isLeft ? "md:flex-row" : "md:flex-row-reverse"
                  }`}
                >
                  {/* Timeline dot */}
                  <div className="absolute left-4 md:left-1/2 w-4 h-4 rounded-full bg-linear-to-br from-primary to-secondary border-4 border-background -translate-x-1/2 z-10 shadow-[0_0_20px_hsl(var(--primary)/0.5)]" />

                  {/* Content card */}
                  <div
                    className={`ml-12 md:ml-0 md:w-[calc(50%-2rem)] ${
                      isLeft ? "md:pr-8" : "md:pl-8"
                    }`}
                  >
                    <div className="group relative rounded-xl border border-border/50 bg-card/50 p-6 backdrop-blur transition-all duration-300 hover:border-primary/50 hover:bg-card/80 hover:shadow-[0_0_30px_hsl(var(--primary)/0.2)]">
                      {/* Gradient accent */}
                      <div className={`absolute top-0 ${isLeft ? "right-0 md:left-0" : "right-0"} w-1 h-full rounded-full bg-linear-to-b from-primary to-secondary opacity-0 group-hover:opacity-100 transition-opacity`} />
                      
                      <div className="flex items-center gap-3 mb-4">
                        <div className="flex items-center justify-center w-10 h-10 rounded-lg bg-linear-to-br from-primary/20 to-secondary/20 text-primary group-hover:from-primary/30 group-hover:to-secondary/30 transition-colors">
                          {chapter.icon}
                        </div>
                        <div>
                          <span className="text-xs font-medium text-secondary uppercase tracking-wider">
                            {chapter.weeks}
                          </span>
                          <h3 className="text-lg font-semibold text-foreground">
                            {chapter.title}
                          </h3>
                        </div>
                      </div>

                      <ul className="space-y-2">
                        {chapter.topics.map((topic, topicIndex) => (
                          <li
                            key={topicIndex}
                            className="flex items-start gap-2 text-sm text-muted-foreground"
                          >
                            <span className="mt-1.5 w-1.5 h-1.5 rounded-full bg-primary/60 shrink-0" />
                            {topic}
                          </li>
                        ))}
                      </ul>
                    </div>
                  </div>

                  {/* Empty space for the other side on desktop */}
                  <div className="hidden md:block md:w-[calc(50%-2rem)]" />
                </div>
              );
            })}
          </div>
        </div>
      </div>
    </section>
  );
};
