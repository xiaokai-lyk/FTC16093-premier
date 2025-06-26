// 页面加载完成后执行
document.addEventListener('DOMContentLoaded', function() {
    // 隐藏加载动画
    setTimeout(() => {
        document.querySelector('.loader').classList.add('hidden');
    }, 500);
    
    // 导航栏滚动效果
    window.addEventListener('scroll', function() {
        const nav = document.querySelector('nav');
        if (window.scrollY > 50) {
            nav.classList.add('scrolled');
        } else {
            nav.classList.remove('scrolled');
        }
    });
    
    // 汉堡菜单切换
    const burger = document.querySelector('.burger');
    const navLinks = document.querySelector('.nav-links');
    
    burger.addEventListener('click', function() {
        navLinks.classList.toggle('active');
        burger.classList.toggle('toggle');
    });
    
    // 滚动动画
    const animateOnScroll = function() {
        const elements = document.querySelectorAll('.section-title, .about-text, .about-image, .sponsor-card, .timeline-item, .gallery-item, .contact-info, .contact-form');
        
        elements.forEach(element => {
            const elementPosition = element.getBoundingClientRect().top;
            const screenPosition = window.innerHeight / 1.3;
            
            if (elementPosition < screenPosition) {
                element.classList.add('animate');
            }
        });
    };
    
    // 初始化滚动动画
    window.addEventListener('scroll', animateOnScroll);
    animateOnScroll(); // 初始检查
    
    // 数字动画
    const counters = document.querySelectorAll('.stat-value');
    const speed = 200;
    
    counters.forEach(counter => {
        const updateCount = () => {
            const target = counter.getAttribute('data-count');
            const count = +counter.innerText;
            
            // 如果目标包含"+"号
            if(target.includes('+')) {
                const num = parseInt(target);
                const inc = num / speed;
                
                if(count < num) {
                    counter.innerText = Math.ceil(count + inc);
                    setTimeout(updateCount, 100);
                } else {
                    counter.innerText = target; // 显示"7+"
                }
            } else {
                // 原来的数字动画逻辑
                const num = +target;
                const inc = num / speed;
                
                if(count < num) {
                    counter.innerText = Math.ceil(count + inc);
                    setTimeout(updateCount, 100);
                } else {
                    counter.innerText = num;
                }
            }
        };
        
        // 当元素进入视口时开始计数
        const observer = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    updateCount();
                    observer.unobserve(counter);
                }
            });
        });
        
        observer.observe(counter);
    });
    
    // 平滑滚动
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function(e) {
            e.preventDefault();
            
            const target = document.querySelector(this.getAttribute('href'));
            if (target) {
                window.scrollTo({
                    top: target.offsetTop - 80,
                    behavior: 'smooth'
                });
                
                // 移动端关闭菜单
                if (navLinks.classList.contains('active')) {
                    navLinks.classList.remove('active');
                    burger.classList.remove('toggle');
                }
            }
        });
    });
});